# -*- coding: utf-8 -*-

import adsk.core
import adsk.fusion
import traceback
import xml.etree.ElementTree as ET
import math
import xml.dom.minidom as DOM
import os


## Transforms a matrix from Fusion 360 to Gazebo.
#
# This transforms a matrix given in the Fusion 360 coordinate system
# to one in the Gazebo cooridnate system.
#
# @param self a matrix given wrt the Fusion 360 coordinate system
# @return the matrix wrt the Gazebo coordinate system
def gazeboMatrix(self):
    matrix = adsk.core.Matrix3D.create()
    matrix.setCell(1, 1, 0)
    matrix.setCell(1, 2, -1)
    matrix.setCell(2, 1, 1)
    matrix.setCell(2, 2, 0)
    self.transformBy(matrix)
    return self

## Converts three double values to string.
#
# This function converts three double values to a string separated by spaces.
#
# @param x the first double value
# @param y the second double value
# @param z the third double value
# @return the string of these values
def vectorToString(x, y, z):
    string = str(x) + " " + str(y) + " " + str(z)
    return string

## Builds SDF pose node from vector.
#
# This function builds the SDF pose node for every joint.
#
# @param vector the vector pointing to the origin of the joint.
# @return the SDF pose node
def sdfPoseVector(vector):
    pose = ET.Element("pose")  
    # convert from cm (Fusion 360) to m (SI)
    x = 0.01 * vector.x
    y = 0.01 * vector.y
    z = 0.01 * vector.z
    pos = vectorToString(x, y, z)
    rot = vectorToString(0, 0, 0)
    pose.text = pos + " " + rot
    return pose
    
## Builds SDF pose node from matrix.
#
# This function builds the SDF pose node for every link.
#
# @param matrix the transformation matrix of the link
# @return the SDF pose node
def sdfPoseMatrix(matrix):
    pose = ET.Element("pose")
    # convert from cm (Fusion 360) to m (SI)
    trans = matrix.translation
    x = 0.01 * trans.x
    y = 0.01 * trans.y
    z = 0.01 * trans.z
    pos = vectorToString(x, y, z)
    # calculate roll pitch yaw from transformation matrix
    r11 = matrix.getCell(0, 0)
    r21 = matrix.getCell(1, 0)
    r31 = matrix.getCell(2, 0)
    r32 = matrix.getCell(2, 1)
    r33 = matrix.getCell(2, 2)
    pitch = math.atan2(-r31, math.sqrt(math.pow(r11, 2) + math.pow(r21, 2)))
    cp = math.cos(pitch)
    yaw = math.atan2(r21 / cp, r11 / cp)
    roll = math.atan2(r32 / cp, r33 / cp)
    rot = vectorToString(roll, pitch, yaw)
    pose.text = pos + " " + rot
    return pose

## Builds SDF inertial node from physical properties.
#
# This function builds the SDF inertial node for every link.
#
# @param physics the physical properties of a link
# @return the SDF inertial node
def sdfInertial(physics, joint_transform):
    inertial = ET.Element("inertial")
    # build pose node of COM
    com = physics.centerOfMass
    com.x = com.x - joint_transform.x
    com.y = com.y - joint_transform.y
    com.z = com.z - joint_transform.z
    pose = sdfPoseVector(com)
    inertial.append(pose)
    # build mass node
    mass = ET.Element("mass")
    mass.text = str(physics.mass)
    inertial.append(mass)
    # build inertia node
    inertia = sdfInertia(physics)
    inertial.append(inertia)
    return inertial

## Builds SDF node for one moment of inertia.
#
# This helper function builds the SDF node for one moment of inertia.
#
# @param tag the tag of the XML node
# @param value the text of the XML node
# @return the SDF moment of inertia node
def sdfMom(tag, value):
    node = ET.Element(tag)
    # convert from kg/cm^2 (Fusion 360) to kg/m^2 (SI)
    node.text = str(0.0001 * value)
    return node

## Builds SDF inertia node from physical properties.
#
# This function builds the SDF inertia node for every link.
#
# @param physics the physical properties of a link
# @return the SDF inertia node
def sdfInertia(physics):
    inertia = ET.Element("inertia")
    (returnValue, xx, yy, zz, xy, yz, xz) = physics.getXYZMomentsOfInertia()
    inertia.append(sdfMom("ixx", xx))
    inertia.append(sdfMom("ixy", xy))
    inertia.append(sdfMom("ixz", xz))
    inertia.append(sdfMom("iyy", yy))
    inertia.append(sdfMom("iyz", yz))
    inertia.append(sdfMom("izz", zz))
    return inertia
    
## Builds SDF link node.
#
# This function builds the SDF link node for every link.
#
# @param lin the link to be exported
# @return the SDF link node
def linkSDF(lin):
    #linkName = lin.component.name
    linkName = clearName(lin.name)
    link = ET.Element("link", name=linkName)
    # build pose node
    matrix = gazeboMatrix(lin.transform)
    pose = sdfPoseMatrix(matrix)
    link.append(pose)
    # get physical properties of occurrence
    physics = lin.physicalProperties
    # build inertial node
    inertial = sdfInertial(physics, lin.transform.translation)
    link.append(inertial)
    # build collision node
    collision = ET.Element("collision", name = linkName + "_collision")
    link.append(collision)
    # build geometry node
    geometry = ET.Element("geometry")
    collision.append(geometry)
    # build mesh node
    mesh = ET.Element("mesh")
    geometry.append(mesh)
    # build uri node
    uri = ET.Element("uri")
    uri.text = "model://meshes/" + clearName(lin.component.name) + ".stl"
    mesh.append(uri)
    # scale the mesh from mm to m
    scale = ET.Element("scale")
    scale.text = "0.001 0.001 0.001"
    mesh.append(scale)
    # build visual node (equal to collision node)
    visual = ET.Element("visual", name = linkName + "_visual")
    visual.append(geometry)
    link.append(visual)
    return link
    
## Builds SDF joint node.
#
# This function builds the SDF joint node for every joint type.
#
# @param joi the joint 
# @param name_parent the name of the parent link
# @param name_child the name of the child link
# @return the SDF joint node
def jointSDF(joi, ui):
    name_parent = clearName(joi.occurrenceOne.name)
    name_child = clearName(joi.occurrenceTwo.name)
    jointInfo = []
    jointType = ""
    jType = joi.jointMotion.jointType
    if jType == 0:
        jointType = "fixed"
    elif jType == 1:
        jointInfo = revoluteJoint(joi)
        jointType = "revolute"
    elif jType == 2:
        jointInfo = sliderJoint(joi)
        jointType = "prismatic"
    elif jType == 3:
        # not implemented
        jointType = ""
    elif jType == 4:
        # not implemented
        jointType = ""
    elif jType == 5:
        # not implemented
        jointType = ""
    elif jType == 6:
        # SDFormat does not implement ball joint limits
        jointType = "ball"
    name = clearName(joi.name)
    joint = ET.Element("joint", name=name, type=jointType)
    # build parent node
    parent = ET.Element("parent")
    parent.text = name_parent
    joint.append(parent)
    # build child node
    child = ET.Element("child")
    child.text = name_child
    joint.append(child)
    # build pose node
    if type(joi.geometryOrOriginOne) == adsk.fusion.JointGeometry:
        origin_vtr = joi.geometryOrOriginOne.origin
        transform_used = joi.occurrenceTwo.transform
        transform_used.invert()
        origin_vtr.transformBy(transform_used)
        
        pose = ET.Element("pose")  
        # convert from cm (Fusion 360) to m (SI)
        pos = vectorToString(origin_vtr.x*0.01, origin_vtr.y*0.01, origin_vtr.z*0.01)
        rot = vectorToString(0, 0, 0)
        pose.text = pos + " " + rot
    else:
        msg = 'Joint '+joi.name+' has an unsupported geometryOrOriginOne type: '+type(joi.geometryOrOriginOne)+', shutting down!'        
        ui.messageBox(msg)
        raise NotImplemented
    joint.append(pose)
    joint.extend(jointInfo)
    return joint

## Builds SDF axis node for revolute joints.
#
# This function builds the SDF axis node for revolute joint.
#
# @param joi one revolute joint object
# @return a list of information nodes (here one axis node)
# for the revolute joint
def revoluteJoint(joi):
    info = []
    # build axis node
    axis = ET.Element("axis")
    xyz = ET.Element("xyz")    
    vector = joi.jointMotion.rotationAxisVector
    xyz.text = vectorToString(vector.x, vector.y, vector.z)
    axis.append(xyz)
    # build limit node
    if joi.jointMotion.rotationLimits.isMaximumValueEnabled or joi.jointMotion.rotationLimits.isMinimumValueEnabled:
        limit = ET.Element("limit")
        axis.append(limit)  
        # Lower and upper limit have to be switched and inverted,
        # because Fusion 360 moves the parent link wrt to the
        # child link and Gazebo moves the child link wrt to the
        # parent link.
        if joi.jointMotion.rotationLimits.isMaximumValueEnabled:
            maxi = joi.jointMotion.rotationLimits.maximumValue
            lower = ET.Element("lower")
            lower.text = str(-maxi)
            limit.append(lower)
        if joi.jointMotion.rotationLimits.isMinimumValueEnabled:
            mini = joi.jointMotion.rotationLimits.minimumValue
            upper = ET.Element("upper")
            upper.text = str(-mini)
            limit.append(upper)

    # build frame node
    frame = ET.Element("use_parent_model_frame")
    frame.text = "0"
    axis.append(frame)
    info.append(axis)
    return info
    
## Builds SDF axis node for slider joints.
#
# This function builds the SDF axis node for slider joint.
#
# @param joi slider joint object
# @return a list of information nodes (here one axis node)
# for the revolute joint
def sliderJoint(joi):
    info = []
    # build axis node
    axis = ET.Element("axis")
    xyz = ET.Element("xyz")  
    vector = joi.jointMotion.slideDirectionVector
    xyz.text = vectorToString(vector.x, vector.y, vector.z)
    axis.append(xyz)
    # build limit node
    if joi.jointMotion.slideLimits.isMaximumValueEnabled or joi.jointMotion.slideLimits.isMinimumValueEnabled:
        limit = ET.Element("limit")
        axis.append(limit)  
        # Lower and upper limit have to be switched and inverted,
        # because Fusion 360 moves the parent link wrt to the
        # child link and Gazebo moves the child link wrt to the
        # parent link.
        if joi.jointMotion.slideLimits.isMaximumValueEnabled:
            maxi = joi.jointMotion.slideLimits.maximumValue
            lower = ET.Element("lower")
            lower.text = str(-maxi)
            limit.append(lower)
        if joi.jointMotion.slideLimits.isMinimumValueEnabled:
            mini = joi.jointMotion.slideLimits.minimumValue
            upper = ET.Element("upper")
            upper.text = str(-mini)
            limit.append(upper)
    # build frame node
    frame = ET.Element("use_parent_model_frame")
    frame.text = "0"
    axis.append(frame)
    info.append(axis)
    return info
    
## Clear filenames of unwanted characters
#
# This function replaces all ':' with underscores and deletes spaces in filenames.
# to one in the Gazebo cooridnate system.
#
# @param name a filename
# @return the filename without ':' and spaces
def clearName(name):
    name = name.replace(":", "_")
    name = name.replace(" ", "")
    name = name.replace("ä", "ae")
    name = name.replace("ü", "ue")
    name = name.replace("ö", "oe")
    name = name.replace("ß", "ss")
    name = name.replace("Ö", "Oe")
    name = name.replace("Ü", "Ue")
    name = name.replace("Ä", "Ae")
    return name
                                    
## Exports a robot model from Fusion 360 to SDFormat.
def run(context):
    title = 'Gazebo SDF exporter'
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        folderDlg = ui.createFolderDialog()
        folderDlg.title = 'Select output folder' 

        # Show folder dialog
        dlgResult = folderDlg.showDialog()
        if dlgResult == adsk.core.DialogResults.DialogOK:
            fileDir = folderDlg.folder
        if fileDir == False:
            ui.messageBox('Gazebo SDF exporter was canceled', title)
            return 0
        
        # uncomment if you want to hardcode an export folder, comment out the dialog above
        #fileDir = "E:\exported"
        
        # get active design   
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        # get root component
        rootComp = design.rootComponent   
        modelName = rootComp.name.split()[0]

        # get all occurrences within the root component
        root = ET.Element("sdf", version="1.6")             
        model = ET.Element("model", name=modelName)         
        root.append(model)                                  
  
        # add static model flag
        static_flag = ET.Element("static")
        static_flag.text = 'false'
        model.append(static_flag)
        
        allJoints = rootComp.allJoints
        allLinks = rootComp.allOccurrences

        for link in allLinks:           
            linkElement = linkSDF(link)
            model.append(linkElement)    
                
        for joi in allJoints:
            jointElement = jointSDF(joi, ui)
            model.append(jointElement)
                
        # leftover rigid-group based code                        
        # for rig in allRigidGroups:   
        #     if rig.name == link_child and rig.name != parent_name :
        #         linkOcc = rigidGroupToSTL(rig)              
        #         link = linkSDF(linkOcc)
        #         model.append(link)                          
        #         #delete the temporary new occurrence
        #         linkOcc.deleteMe()
        #         #Call doEvents to give Fusion a chance to react.
        #         adsk.doEvents()
        #         export_next(link_child, terminate)
        #     elif rig.name == link_parent and rig.name != parent_name:
        #         # jointsList.append(link_child)
        #         #export_next(link_child, linksList)
        #         linkOcc = rigidGroupToSTL(rig)    
        #         link = linkSDF(linkOcc)
        #         model.append(link)                          
        #         #delete the temporary new occurrence
        #         linkOcc.deleteMe()
        #         #Call doEvents to give Fusion a chance to react.
        #         adsk.doEvents()
        #         print("exported link ", rig.name)
        #         export_next(link_parent, terminate)
               
        # create a single exportManager instance
        exportMgr = design.exportManager
        # get the script location
        try: os.mkdir(fileDir + '/meshes')
        except: pass
        scriptDir = fileDir + '/meshes'  
        
        allOccus = rootComp.allOccurrences
        for occ in allOccus:
            try:
                print(occ.component.name)
                fileName = scriptDir + "/" + clearName(occ.component.name)              
                # create stl exportOptions
                stlExportOptions = exportMgr.createSTLExportOptions(occ, fileName)
                stlExportOptions.sendToPrintUtility = False
                stlExportOptions.isBinaryFormat = False
                exportMgr.execute(stlExportOptions)
            except:
                ui.messageBox('Component ' + clearName(occ.component.name) + 'is broken.')
                    
        
        # get all construction points that serve as viaPoints
        filename = fileDir + "/model.sdf"
        domxml = DOM.parseString(ET.tostring(root))
        pretty = domxml.toprettyxml()
        file = open(filename, "w")
        file.write(pretty)
        file.close()
        ui.messageBox("SDF file of model " + modelName + " written to '" + fileDir + "'.")
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc())) 