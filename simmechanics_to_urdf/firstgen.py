#!/usr/bin/python

import sys
import os

import xml.dom.minidom
import lxml.etree
import math
import numpy    #for creating matrix to convert to quaternion
import yaml
import urdf_parser_py.urdf
import argparse
import csv

# Conversion Factors
INCH2METER = 0.0254
SLUG2KG = 14.5939029
SLUGININ2KGMM = .009415402
MM2M = .001

# Special Reference Frame(s)
WORLD = "WORLD"

# Arbitrary List of colors to give pieces different looks
COLORS =[("green", (0, 1, 0, 1)), ("black", (0, 0, 0, 1)), ("red", (1, 0, 0, 1)),
     ("blue", (0, 0, 1, 1)), ("yellow", (1, 1, 0, 1)), ("pink", (1, 0, 1, 1)),
     ("cyan", (0, 1, 1, 1)), ("green", (0, 1, 0, 1)), ("white", (1, 1, 1, 1)),
     ("dblue", (0, 0, .8, 1)), ("dgreen", (.1, .8, .1, 1)), ("gray", (.5, .5, .5, 1))]

# epsilon for testing whether a number is close to zero
_EPS = numpy.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())
def euler_from_matrix(matrix, axes='sxyz'):
    """Return Euler angles from rotation matrix for specified axis sequence.

axes : One of 24 axis sequences as string or encoded tuple

Note that many Euler angle triplets can describe one matrix.

>>> R0 = euler_matrix(1, 2, 3, 'syxz')
>>> al, be, ga = euler_from_matrix(R0, 'syxz')
>>> R1 = euler_matrix(al, be, ga, 'syxz')
>>> numpy.allclose(R0, R1)
True
>>> angles = (4.0*math.pi) * (numpy.random.random(3) - 0.5)
>>> for axes in _AXES2TUPLE.keys():
... R0 = euler_matrix(axes=axes, *angles)
... R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
... if not numpy.allclose(R0, R1): print axes, "failed"

"""
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j], M[i, k])
            ay = math.atan2( sy, M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k], M[j, j])
            ay = math.atan2( sy, M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j], M[k, k])
            ay = math.atan2(-M[k, i], cy)
            az = math.atan2( M[j, i], M[i, i])
        else:
            ax = math.atan2(-M[j, k], M[j, j])
            ay = math.atan2(-M[k, i], cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az


def euler_from_quaternion(quaternion, axes='sxyz'):
    """Return Euler angles from quaternion for specified axis sequence.

    >>> angles = euler_from_quaternion([0.06146124, 0, 0, 0.99810947])
    >>> numpy.allclose(angles, [0.123, 0, 0])
    True
    """
    return euler_from_matrix(quaternion_matrix(quaternion), axes)



class Converter:
    def __init__(self):
        # initialize member variables
        self.links = {}
        self.frames = {}
        self.joints = {}
        self.names = {}
        self.colormap = {}
        self.colorindex = 0
        self.usedcolors = {}
        self.ref2nameMap = {}
        self.realRootLink = None
        # map mapping USERADDED linkName+displayName to fid of the frame
        self.linkNameDisplayName2fid = {}

        #Start the Custom Transform Manager
        self.tfman = CustomTransformManager()

        # Extra Transforms for Debugging
        self.tfman.add([0,0,0], [0.70682518,0,0,0.70682518], "ROOT", WORLD) # rotate so Z axis is up

    def convert(self, filename, yaml_configfile, csv_configfile, mode):
        self.mode = mode

        # Parse the global YAML configuration file
        self.parseYAMLConfig(yaml_configfile)

        # Parse the joint CSV configuratio nfile
        self.parseJointCSVConfig(csv_configfile)

        # Parse the input file
        self.parse(xml.dom.minidom.parse(filename))
        self.buildTree(self.root)

        # Create the output
        self.output(self.root)

        # output the output
        if mode == "xml":
            #print("URDF model to print : \n " + str(self.result) + "\n" )
            self.generateXML()
            self.addSensors()
            self.addXMLBlobs()
            print(lxml.etree.tostring(self.urdf_xml,pretty_print=True))
        if mode == "graph":
            print(self.graph())
        if mode == "debug":
            self.debugPrints()

    def debugPrints(self):
        print("root_link_T_l_foot_CG :\n " + str(self.tfman.getHomTransform("X"+"root_link","l_foot"+"CG")[:3,3])); 
        print("root_link_T_r_foot_CG :\n " + str(self.tfman.getHomTransform("X"+"root_link","r_foot"+"CG")[:3,3])); 

    def generateXML(self):
        self.urdf_xml = self.result.to_xml();

    def addSensors(self):
        generator = URDFGazeboSensorsGenerator();
        # for the ft sensors, load the sensors as described in the YAML without further check
        for ftSens in self.forceTorqueSensors:
            referenceJoint = ftSens["jointName"];
            if( 'sensorName' not in ftSens.keys() ):
                sensorName = referenceJoint
            else:
                sensorName = ftSens["sensorName"];
            ft_el = generator.getURDFForceTorque(referenceJoint,sensorName,ftSens["directionChildToParent"])
            self.urdf_xml.append(ft_el);
        
        # for the IMU, we rely on pose given by a USERADDED frame 
        for imuSens in self.IMUs: 
            sensorLink = imuSens["linkName"]; 
            frameName = imuSens.get("frameName");
            referenceLink = imuSens.get("frameReferenceLink"); 
            sensorName    = imuSens.get("sensorName");

            if( frameName is None ):
                # If frame is not specified, the sensor frame is the link frame 
                offset = [0.0, 0.0, 0.0]; 
                rot    = quaternion_from_matrix(numpy.identity(4));
            else:
                # The default referenceLink is the sensorLink itself
                if( referenceLink is None): 
                    referenceLink = sensorLink; 
                
                # Get user added frame 
                sensor_frame_fid = self.linkNameDisplayName2fid[ (referenceLink,frameName) ];
                (offset, rot) = self.tfman.get( "X"+ sensorLink , sensor_frame_fid)

            if( sensorName is None): 
                sensorName = sensorLink + "_" + frameName;
            
           
            pose = toGazeboPose(offset,rot); 

            #sys.stderr.write("Processing link " + link['uid'] + "\n")

            imu_el =  generator.getURDFIMU(sensorLink,sensorName,pose)
 
            self.urdf_xml.append(imu_el);

    def addXMLBlobs(self):
        for blob in self.XMLBlobs:
            if not( blob is None or blob is ''):
                blob_el = lxml.etree.fromstring(blob);
                self.urdf_xml.append(blob_el);
            else:
                sys.stderr.write("Warning: malformed XMLBlob: " + blob + "\n")
                sys.stderr.write("Ingnoring it")

    def parseYAMLConfig(self, configFile):
        """Parse the YAML configuration File, if it exists.
           Set the fields the default if the config does
           not set them """
        if configFile == None:
            configuration = {}
        else:
            configuration = yaml.load(open(configFile, 'r'))
            if configuration == None:
                configuration = {}

        self.freezeList = []
        self.redefinedjoints = {}

        self.root = configuration.get('root', None)
        self.extrajoints = configuration.get('extrajoints', {})
        self.extraframes = []
        self.filenameformat = configuration.get('filenameformat', None)
        self.filenameformatchangeext = configuration.get('filenameformatchangeext', None)
        if( self.filenameformat is None and self.filenameformatchangeext is None ):
            # if neither filenameformat nor filenameformatchangeext is defined, use the default
            self.filenameformat = '%s'
        if( self.filenameformat is not None and self.filenameformatchangeext is not None ):
            #    
            print("Error: both filenameformat and filenameformatchangeext are defined")
            assert(False)

        self.forcelowercase = configuration.get('forcelowercase', True)

        ## Frames related options
        
        # If the exportAllUserAdded frame is setted, export all USERADDED frames
        self.exportAllUseradded = configuration.get('exportAllUseradded', False)

        # Load the linkFrames 
        self.linkFrames = configuration.get('linkFrames',[])
        self.linkFramesMap = {}
        for link_frame in self.linkFrames: 
            # add default frameReferenceLink if not included
            if( link_frame.get("frameReferenceLink") is None ):
                 link_frame["frameReferenceLink"] = link_frame["linkName"];
            self.linkFramesMap[link_frame["linkName"]] = link_frame;

        # Get a list of sensors 
        self.forceTorqueSensors = configuration.get('forceTorqueSensors',{});
        self.IMUs = configuration.get('IMUs',{});

        # Load the exported frames 
        exportedFrames = configuration.get('exportedFrames',[])
        self.exportedFramesMap = {}
        for exported_frame in exportedFrames:
            if( exported_frame.get("frameReferenceLink") is not None ):
                self.exportedFramesMap[(exported_frame["frameReferenceLink"],exported_frame["frameName"])] = exported_frame;
            else:
                # if the frameReferenceLink is missing, just add the export_frame dict using only the frameName as key: 
                # we will add the full tutple (frameReferenceLink,frameName) later
                self.exportedFramesMap[exported_frame["frameName"]] = exported_frame;

        # Augment the exported frames with sensors for which the exportFrameInURDF option is enabled 
        #for ftSens in self.forceTorqueSensors:
        #    if( ftSens["exportFrameInURDF"] ): 
        #        exported_frame = {}
        #        exported_frame["frameName"]
            
 
        for imuSens in self.IMUs:
            if( imuSens["exportFrameInURDF"] ): 
                exported_frame = {}
                exported_frame["frameName"] = imuSens["frameName"] 
                if( imuSens.get("exportedFrameName") is not None ): 
                    exported_frame["exportedFrameName"] = imuSens["exportedFrameName"];
                else:
                    exported_frame["exportedFrameName"] = imuSens["sensorName"];

                if( imuSens.get("frameReferenceLink") is not None ):
                    exported_frame["frameReferenceLink"] = imusens["frameReferenceLink"];
                else:
                    exported_frame["frameReferenceLink"] = imuSens["linkName"];

                self.exportedFramesMap[(exported_frame["frameReferenceLink"],exported_frame["frameName"])] = exported_frame; 
            
 
        # Load scales options
        scale_str = configuration.get('scale', None)
        if( scale_str is not None ):
            self.scale = [float(scale_el) for scale_el in scale_str.split()]
        else:
            self.scale = None
        self.freezeAll = configuration.get('freezeAll', False)
        self.baseframe = configuration.get('baseframe', WORLD)
        self.damping_fallback = configuration.get('damping',0.1)
        self.friction_fallback = configuration.get('friction',None)

        self.effort_limit_fallback = configuration.get('effort_limit',50000)
        self.velocity_limit_fallback = configuration.get('velocity_limit',50000)
                     
        self.rename = configuration.get('rename',{})

        # Get lists converted to strings
        self.removeList = configuration.get('remove', {})
        self.freezeList = [ str(e) for e in configuration.get('freeze', []) ]

        # Get map with key converted to strings
        jointmap = configuration.get('redefinedjoints', {})
        for x in jointmap.keys():
            self.redefinedjoints[str(x)] = jointmap[x]

        # Add Extra Frames
        for frame in configuration.get('moreframes', []):
            self.tfman.add(frame['offset'], frame['orientation'], frame['parent'], frame['child'])

        # SimMechanics bug inertia workaround
        mirroredInertia = configuration.get('mirroredInertia',[])
        self.mirroredInertiaMap = {}
        for mirrored_link_dict in mirroredInertia:
            mirroredLink = mirrored_link_dict["mirroredLink"]
            self.mirroredInertiaMap[mirroredLink] = mirrored_link_dict;
 
        self.inertiaWorkaround = configuration.get('inertiaWorkaround',None);
        if( self.inertiaWorkaround is not None ):
            self.mirroredLinks = self.inertiaWorkaround["mirroredLinks"].split()
        else: 
            self.mirroredLinks = None;

        # Get a list of joints for which we want to invert the rotation axis direction
        self.reverseRotationAxis = configuration.get('reverseRotationAxis',[]);

        # Get a list of blob of XML tags to add to the URDF
        self.XMLBlobs = configuration.get("XMLBlobs",[])


    def parseJointCSVConfig(self, configFile):
        """Parse the CSV configuration File, if it exists."""
        self.joint_configuration = {}
        if configFile is not None:
            with open(configFile, 'r') as csvfile:
                my_dialect = csv.Sniffer().sniff(csvfile.read(1024))
                csvfile.seek(0)
                reader = csv.DictReader(csvfile, dialect=my_dialect)
                for row in reader:
                    self.joint_configuration[row["joint_name"]] = row
        

    def parse(self, element):
        """Recursively goes through all XML elements
           and branches off for important elements"""
        name = element.localName
        # Grab name from root element AND recursively parse
        if name == "PhysicalModelingXMLFile":
            dict = getDictionary(element)
            self.name = dict['name']

        if name == "Body":
            self.parseLink(element)
        elif name == "SimpleJoint":
            self.parseJoint(element)
        elif name == "Ground":
            dict = getDictionary(element)
            self.parseFrames(dict['frame'], "GROUND")
        else:
            for child in element.childNodes:
                self.parse(child)

    def parseLink(self, link):
        """Parse the important bits of a link element"""
        linkdict = getDictionary(link)
        uid = self.getName(linkdict['name'])
        linkdict['uid']       = uid;
        linkdict['neighbors'] = []
        linkdict['children'] = []
        linkdict['jointmap'] = {}

        # Save the frames for separate parsing
        frames = linkdict['frames']
        linkdict['frames'] = None

        # Save the color if it exists
        if 'MaterialProp' in linkdict:
            colorelement = linkdict['MaterialProp'][1]
            color = colorelement.childNodes[0].data
            linkdict['MaterialProp'] = None
            linkdict['color'] = list(map(float, color.split(","))) + [1.0]

        self.links[uid] = linkdict
        self.parseFrames(frames, uid)

        # Save First Actual Element as Root, if not defined already
        if self.root == None and "geometryFileName" in linkdict:
            self.root = uid

    def parseFrames(self, frames, parent_link):
        """Parse the frames from xml"""
        for frame in frames:
            if frame.nodeType is frame.TEXT_NODE:
                continue
            fdict = getDictionary(frame)
            # We don't identify frames with ref attribute because USERADDED
            # frames don't have ref attributes. We always use instead the
            # urdf_link + name scheme (note that for added frames simmechanics link
            # is different from urdf_link
            fid = parent_link +  fdict['name']

            # for using the ref numbers of the frame we build a map from the ref numbers 
            # to the names
            # fid = str(frame.getAttribute("ref"))
            ref = str(frame.getAttribute("ref"))
            
            self.ref2nameMap[ref] =  fid;

            fdict['parent'] = parent_link

            offset = getlist(fdict['position'])

            # for models mirrored in Creo there is a strange bug 
            # in SimMechanics Link that causes the inertia of the 
            # mirrored links to be wrongly placed. We add here 
            # some specific workaround for those link, if this 
            # is requested in the YAML file
            if( self.mirroredLinks is not None ):
                if( fdict['name'] == 'CG' and parent_link in self.mirroredLinks ):
                    offset[2] = -offset[2]; 
           
            units = fdict['positionUnits']
            for i in range(0, len(offset)):
                offset[i] = convert(offset[i], units)

            orientation = getlist(fdict['orientation'])
            quat = matrixToQuaternion(orientation)
            # If the frame does not have a reference number,
            # use the name plus a suffix (for CG or CS1...
            # If the frame does not have a reference number,
            # but it is a USERADDED frame (frame added on the CAD
            # for export in simmechanics) and the exportAllUserAdded 
            # option is set to True, export the frame using the displayName tag 
            # otherwise ignore the frame
            if fdict['nodeID'].endswith('(USERADDED)'):
                useradded_frame_name = fdict['displayName']

                # clean all possible exportedFrames that were missing the frameReferenceLink option
                if useradded_frame_name in self.exportedFramesMap.keys():
                    buf_export_frame = self.exportedFramesMap[useradded_frame_name] 
                    buf_export_frame["frameRefenceLink"] = parent_link;
                    self.exportedFramesMap[(parent_link,useradded_frame_name)] = buf_export_frame; 
 
                # Frame is added if exportAllUseradded is setted or 
                # if frame is part of exportedFrames structure 
                if self.exportAllUseradded or () or ((parent_link,useradded_frame_name) in self.exportedFramesMap.keys()):
                    map_key = (parent_link,useradded_frame_name);
                    if( map_key in self.exportedFramesMap.keys() ):
                        if( "exportedFrameName" in self.exportedFramesMap[map_key].keys() ):
                            useradded_frame_name = self.exportedFramesMap[map_key]["exportedFrameName"];

                    fid = useradded_frame_name + "CS1"
                    extraframe = {'parentlink':parent_link,'framename':useradded_frame_name}
                    self.extraframes = self.extraframes + [extraframe]
                    #add link to self.links structure
                    linkdict = {}
                    linkdict['name'] = useradded_frame_name
                    fdict['parent'] = useradded_frame_name
                    linkdict['neighbors'] = []
                    linkdict['children'] = []
                    linkdict['jointmap'] = {}
                    linkdict['frames'] = None
                    linkdict['uid'] = linkdict['name']
                    self.links[useradded_frame_name] = linkdict

                # Storing the displayName to the fid of the frame, to retrive the USERADDED frame when assigning link frames
                self.linkNameDisplayName2fid[(parent_link,fdict['displayName'])] = fid;           
		  
                       

            self.tfman.add(offset, quat, WORLD, fid)
            self.frames[fid] = fdict

    def parseJoint(self, element):
        """Parse the joint from xml"""
        dict = getDictionary(element)
        joint = {}
        joint['name'] = dict['name']
        uid = self.getName(joint['name'])

        frames = element.getElementsByTagName("Frame")
        ref_parent = str(frames[0].getAttribute("ref"))
        ref_child  = str(frames[1].getAttribute("ref"))

        joint['ref_parent'] = ref_parent
        joint['ref_child'] = ref_child
        type = element.getElementsByTagName("Primitive")

        # If there multiple elements, assume a fixed joint
        # \todo TODO fix and remove this assumption
        if len(type)==1:
            pdict = getDictionary(type[0])
            joint['type'] = pdict['name']
            joint['axis'] = pdict['axis']
            joint['axisReferenceFrame'] = pdict['referenceFrame']
            if joint['type'] == 'weld':
                joint['type'] = 'fixed'
        else:
            joint['type'] = 'fixed'

        # Ignore joints on the remove list
        #print("Parsing joint " + joint['name'])
        #print("Removelist: ")
        #print(self.removeList)
        if (uid in self.removeList) or (joint['name'] in self.removeList):
            #print(joint['name']+" is in removelist")
            return

        # Force joints to be fixed on the freezeList
        if (uid in self.freezeList) or (joint['name'] in self.freezeList) or self.freezeAll:
            joint['type'] = 'fixed'

        # Redefine specified joints on redefined list
        if joint['ref_parent'] in self.redefinedjoints.keys():
            jdict = self.redefinedjoints[joint['ref_parent']]
            if 'name' in jdict:
                uid = jdict['name']

            # Default to continuous joints
            joint['type'] = jdict.get('type', 'continuous')
            
            if 'axis' in jdict:
                #print("axis" + str(jdict['axis']))
                joint['axis'] = jdict['axis']
                joint['axisReferenceFrame'] = jdict['referenceFrame']
            if 'limits' in jdict:
                joint['limits'] = jdict['limits']

        # If some limits are defined in the CSV joint configuration file load them
       

        #if the joint is revolute but no limits are defined, switch to continuous
        if 'limits' not in joint.keys()  and joint['type'] == "revolute":
            joint['type'] = "continuous";


        self.joints[uid] = joint

    def buildTree(self, root):
        """Reduce the graph structure of links and joints to a tree
           by breadth first search. Then construct new coordinate frames
           from new tree structure"""

        #resolve the undefined reference for all the joints 
        for jid in self.joints:
            jointdict = self.joints[jid]
            if( 'parent' not in jointdict.keys() ):
                jointdict['parent'] = self.ref2nameMap[jointdict['ref_parent']]
            if( 'child' not in jointdict.keys() ):
                jointdict['child'] = self.ref2nameMap[jointdict['ref_child']]

            # Find the real rootLink 
            if( jointdict['parent'].startswith('RootPart') and jointdict['type'] == 'fixed' ):
                if( self.realRootLink is not None ):
                    pass
                    #print("[WARN] multiple links attached to the RootPart, please open an issue with your model")
                    #print("       at https://github.com/robotology-playground/simmechanics-to-urdf/issues/new")
                else:
                    self.realRootLink = self.getLinkNameByFrame(jointdict['child'])
        
        # Some postprocessing that was not possible to do while parsing
        for extraframe in self.extraframes:
            pid = extraframe['parentlink']
            # Some USERADDED frames could be attached to the dummy root RootPart 
            # In this case substitute the dummy root with the real first link 
            # attached to the RootPart with a fixed link
            if pid == 'RootPart':

                extraframe['parentlink'] = self.realRootLink
                # notice that we can disregard the original parent frame 
                # of the joint because it is a fixed joint

        # Add necessary information for any frame in the SimMechanics XML
        # file that we want to save to URDF. Given that URDF does not 
        # have a frame concept at all, we are bound to create "dummy" 
        # links and joints to express the notion of frames 
        for extraframe in self.extraframes:
            pid = extraframe['parentlink']
            cid = extraframe['framename']

            joint_name = cid + "_fixed_joint"; 

            self.links[pid]['neighbors'].append(cid)
            self.links[pid]['jointmap'][cid] = joint_name
            self.links[cid]['neighbors'].append(pid)
            self.links[cid]['jointmap'][pid] = joint_name
            self.joints[joint_name] = {'name': joint_name, 'parent': pid + 'CS1', 'child': cid + 'CS1', 'type': 'fixed'}
            #for (k,v) in extraframe['attributes'].items():
            #    self.links[cid][k] = v


        # Create a list of all neighboring links at each link
        for jid in self.joints:
            jointdict = self.joints[jid]
            if "Root" in jointdict['name']:
                continue
            pid = self.getLinkNameByFrame(jointdict['parent'])
            cid = self.getLinkNameByFrame(jointdict['child'])
            parent = self.links[pid]
            child = self.links[cid] 
            
            parent['neighbors'].append(cid)
            parent['jointmap'][cid] = jid
            child['neighbors'].append(pid)
            child['jointmap'][pid] = jid

        #import pprint
        #pp = pprint.PrettyPrinter(indent=4)
        #pp.pprint(self.joints)


        # Add necessary information for any user-defined joints
        for (name, extrajoint) in self.extrajoints.items():
            pid = extrajoint['pid']
            cid = extrajoint['cid']
            jorigin = extrajoint['jorigin']
            newframe = name + "_frame"

            self.links[pid]['neighbors'].append(cid)
            self.links[pid]['jointmap'][cid] = name
            self.links[cid]['neighbors'].append(pid)
            self.links[cid]['jointmap'][pid] = name
            self.joints[name] = {'name': name, 'parent': jorigin, 'child': newframe}
            for (k,v) in extrajoint['attributes'].items():
                self.joints[name][k] = v
            self.frames[jorigin] = {'parent': pid}
            self.frames[newframe] = {'parent': cid}

        # Starting with designated root node, perform BFS to
        # create the tree
        queue = [ root ]
        self.links[root]['parent'] = "GROUND"
        while len(queue) > 0:
            id = queue.pop(0)
            link = self.links[id]
            for n in link['neighbors']:
                nbor = self.links[n]
                # if a neighbor has not been visited yet,
                # add it as a child node
                if not 'parent' in nbor:
                    nbor['parent'] = id
                    queue.append(n)
                    link['children'].append(n)

        # build new link coordinate frames
        # URDF has the unconvenient requirement that the link frame
        # origin should be placed in the axis of the parent joint, 
        # so we have to place special care in 
        for id in self.links:
 
            link = self.links[id]
            if not 'parent' in link:
                continue
            parentid = link['parent']
            jointIsNotFixed = True
            if parentid == "GROUND":
                ref = self.baseframe
            else:
                joint = self.joints[link['jointmap'][parentid]]
                ref = joint['parent']
                if( joint['type'] == 'fixed' ):
                    jointIsNotFixed = False
            # If a frame is not fixed, then the default link frame
            # has the orientation of the link "CS1" frame and the origin 
            # of the joint. 
            # However using the linkFrames options is possible to use an 
            # USERADDED frame as the linkFrame. Given that the URDF enforces
            # the link frame origin to lay on the axis of parent joint, the USERADDED
            # frame is used unmodified only if its origin lays on joint axis.
            # Otherwise the rotation of the frame will be left unchanged, and a new origin 
            # will be found as the projection of the USERADDED frame origin to the axis (i.e. 
            # the point on the axis closest to the USERADDED frame origin).
            if( jointIsNotFixed and not (parentid == "GROUND")  ): 
                if( self.linkFramesMap.get(link['uid']) is None ):
                    # no frame redefinition 
               	    (off1, rot1) = self.tfman.get(WORLD, ref)
            	    (off2, rot2) = self.tfman.get(WORLD, id + "CS1")
            	    self.tfman.add(off1, rot2, WORLD, "X" + id)
                else: 
                    # using a useradded frame
                    new_link_frame_fid = self.linkNameDisplayName2fid[ (self.linkFramesMap[link['uid']]["frameReferenceLink"],self.linkFramesMap[link['uid']]["frameName"])];
                    (new_link_frame_off, new_link_frame_rot) = self.tfman.get(WORLD,new_link_frame_fid)
               	    (joint_offset, joint_rot) = self.tfman.get(WORLD, ref)
                    # get axis for the parent joint
                    jointdict = self.joints[link['jointmap'][parentid]]
                    axis_string = jointdict['axis'].replace(',', ' ')
                    axis = [float(axis_el) for axis_el in axis_string.split()]
                     
                    axis_np = numpy.array(axis) 
                    joint_offset_np = numpy.array(joint_offset)
                    new_link_frame_off_np = numpy.array(new_link_frame_off);
                    
                    axis_normalized = axis_np/numpy.linalg.norm(axis_np);
                     
                    # Projection math: project the frame origin on the joint axis
                    new_link_frame_off_projected = numpy.dot(new_link_frame_off_np-joint_offset_np,axis_normalized)*axis_normalized+joint_offset_np;

            	    self.tfman.add(new_link_frame_off_projected, new_link_frame_rot, WORLD, "X" + id)
                      
            else:
                # If the parent joint is fixed, the URDF format does not 
                # default we use enforce any constraint on the frame placement 
                # and we use the id+"CS1" frame as the link frame. 
                # The  frame of the link attached with a fixed joint 
                # can be optionally set to a USERADDED frame using the 
                # linkFrames options
                #print(str(link))
                #print("Link " + str(link['uid']) + " has a fixed joint parent");
                if( link['uid'] in self.linkFramesMap.keys() ):
                    #print(" Using " + self.linkFramesMap[link['uid']]["frame"] + " of link " + self.linkFramesMap[link['uid']]["frameReferenceLink"] + " as link frame for " + link['uid'])
                    #print(str(self.linkNameDisplayName2fid));
                    new_link_frame_fid = self.linkNameDisplayName2fid[ (self.linkFramesMap[link['uid']]["frameReferenceLink"],self.linkFramesMap[link['uid']]["frameName"])];
                    (off, rot) = self.tfman.get(WORLD,new_link_frame_fid)
                    self.tfman.add(off, rot, WORLD, "X" + id)
                else:
                    if( parentid == "GROUND" ):
                        #be consistent with the old behaviour
                        (off1, rot1) = self.tfman.get(WORLD, ref)
            	        (off2, rot2) = self.tfman.get(WORLD, id + "CS1")
            	        self.tfman.add(off1, rot2, WORLD, "X" + id)
                    else:
                        # If nothing special happens, use CS1 as link frame
                        (off, rot) = self.tfman.get(WORLD, id + "CS1")
                        self.tfman.add(off, rot, WORLD, "X" + id)


    def output(self, rootid):
        """Creates the URDF from the parsed document.
           Makes the document and starts the recursive build process"""
        self.result = urdf_parser_py.urdf.URDF(self.name)
        self.outputLink(rootid)
        self.processLink(rootid)

    def processLink(self, id):
        """ Creates the output for the specified node's
            child links, the connecting joints, then
            recursively processes each child """
        link = self.links[id]
        for cid in link['children']:
            jid = link['jointmap'][cid]

            self.outputLink(cid)
            self.outputJoint(jid, id)
            self.processLink(cid)

    def isValidInertiaMatrix(self, inertia, tol):
        """Check that a matrix is a valid inertia matrix: 
           * Check simmetry
           * Check positive definitess 
           * Check triangular inequality of eigen values"""
        # Check simmetry 
        deter = numpy.linalg.det(inertia)
        if( abs((inertia[0,1]-inertia[1,0])/deter) > tol or 
            abs((inertia[0,2]-inertia[2,0])/deter) > tol or 
            abs((inertia[1,2]-inertia[2,1])/deter) > tol ):
            sys.stderr.write("Inertia: " + str(inertia) + " is not a valid Inertia matrix\n");
            return False;

        # Compute eigenvalues 
        [s,v] = numpy.linalg.eig(inertia) 
        if( (s[0])/deter < tol or 
            (s[1])/deter < tol or 
            (s[2])/deter < tol ):
            sys.stderr.write("Inertia: " + str(inertia) + " is not a valid Inertia matrix\n");
            return False;
 
        # Check triangle inequality 
        if( ((s[0]+s[1]-s[2])/deter < tol) or 
            ((s[1]+s[2]-s[0])/deter < tol) or
            ((s[0]+s[2]-s[1])/deter < tol) ):
            sys.stderr.write("Inertia: " + str(inertia) + " is not a valid Inertia matrix\n");
            return False;
        
        return True;

    def outputLink(self, id):
        """ Creates the URDF output for a single link """

        linkdict = self.links[id]
        if linkdict['name'] == "RootPart":
            return


        if( 'geometryFileName' in linkdict.keys() ) :
            ##############################################################
            ### Define Geometry (if this is not a fake link, i.e. a frame)
            ##############################################################
            visual = urdf_parser_py.urdf.Visual()
            collision = urdf_parser_py.urdf.Collision()

            filename = linkdict['geometryFileName']
            if self.forcelowercase:
                filename = filename.lower()

            if ( self.filenameformat is not None ):
                filename = self.filenameformat % filename
            else:
                filenameNoExt =  os.path.splitext(filename)[0]
                filename = self.filenameformatchangeext % filenameNoExt

            visual.geometry = urdf_parser_py.urdf.Mesh(filename, self.scale)
            collision.geometry = visual.geometry

            # Visual offset is difference between origin and CS1
            (off, rot) = self.tfman.get("X" + id, id+"CS1")
            rpy = list(euler_from_quaternion(rot))
            visual.origin = urdf_parser_py.urdf.Pose(zero(off), zero(rpy))
            collision.origin = visual.origin

            # Define Material
            visual.material = urdf_parser_py.urdf.Material()
            # Use specified color, if exists. Otherwise, get random color
            if 'color' in linkdict:
                cname = "%s_color"%id
                (r,g,b,a) = linkdict['color']
            else:
                (cname, (r,g,b,a)) = self.getColor(linkdict['name'])

            visual.material.name = cname

            # If color has already been output, only output name
            if not cname in self.usedcolors:
                visual.material.color = urdf_parser_py.urdf.Color(r,g,b,a)
                self.usedcolors[cname] = True
                
        else:
            visual = None
            collision = None

            ##############################################################
            ### Define Inertial Frame and inertia informations (if this is not a fake link, i.e. a frame)
            ##############################################################
        if( 'mass' in linkdict.keys() ) :

            inertial = urdf_parser_py.urdf.Inertial()

            if( id not in self.mirroredInertiaMap ): 
                # usual: get inertia informations from SimMechanics XML
                units = linkdict['massUnits']
                massval = convert(float(linkdict['mass']), units)
                inertial.mass = massval

                matrix = getlist(linkdict["inertia"])

                units = linkdict['inertiaUnits']

                for i in range(0,len(matrix)):
                    matrix[i] = convert(matrix[i], units)

                inertial.inertia = urdf_parser_py.urdf.Inertia()
                inertial.inertia.ixx = matrix[0]
                inertial.inertia.ixy = matrix[1]
                inertial.inertia.ixz = matrix[2]
                inertial.inertia.iyy = matrix[4]
                inertial.inertia.iyz = matrix[5]
                inertial.inertia.izz = matrix[8]

                # Inertial origin is the center of gravity
                (off, rot) = self.tfman.get("X" + id, id+"CG")
                #print("X"+id+" to "+id+"CG:")
                #print(off)
                #print(rot)
                rpy = list(euler_from_quaternion(rot))

                inertial.origin = urdf_parser_py.urdf.Pose(zero(off), zero(rpy))
            else: 
                # if link is mirroredInertia, we should not trust the inertial infromation 
                # provided by SimMechanics XML because it could be buggy. We will mirror  
                # the inertia information of another link instead
                mirroredLink = id
                originalLink = self.mirroredInertiaMap[mirroredLink]["originalLink"];
                simmetryReferenceLink = self.mirroredInertiaMap[mirroredLink]["simmetryReferenceLink"];
                symmetryPlane = self.mirroredInertiaMap[mirroredLink]["symmetryPlane"]

                if( not( symmetryPlane == "xz" ) ):
                    print("simmechanics_to_urdf: only xz symmetryPlane is supported, please file an issue at https://github.com/robotology-playground/simmechanics-to-urdf/issues/new to get more symmetryPlane supported.");
                    assert(False);

                originalLinkDict = self.links[originalLink]
        
                # Mass: the mass is simply copied by the original link 
                units = originalLinkDict['massUnits']
                massval = convert(float(originalLinkDict['mass']), units)
                inertial.mass = massval

                # COM: we have to express the COM in simmetryReferenceLink, 
                # mirror it and then express it in mirroredLink frame
                # T is a 4x4 homogeneous matrix
                # Get {}^simmetryReferenceLink COM
                (off, rot) = self.tfman.get("X" + simmetryReferenceLink, originalLink+"CG")
                simmetryRefenceLink_COM = numpy.zeros(4)
                simmetryRefenceLink_COM[0] = off[0]
                simmetryRefenceLink_COM[1] = off[1]
                simmetryRefenceLink_COM[2] = off[2]
                simmetryRefenceLink_COM[3] = 1;
 
                # Get {}^mirroredLink T_simmetryReferenceLink  
                mirroredLink_T_simmetryReferenceLink = self.tfman.getHomTransform("X" + mirroredLink, "X" + simmetryReferenceLink)

                # xz simmetry : y --> -y 
                simmetryRefenceLink_COM[1] = -simmetryRefenceLink_COM[1];
 
                # {}^mirroredLink COM = {}^mirroredLink T_simmetryReferenceLink {}^simmetryReferenceLink COM
                mirroredLink_COM = numpy.dot(mirroredLink_T_simmetryReferenceLink,simmetryRefenceLink_COM);

                off = [0.0,0.0,0.0];
                off[0] = mirroredLink_COM[0];
                off[1] = mirroredLink_COM[1];
                off[2] = mirroredLink_COM[2];
                
                # Inertia: the inertia both in SimMechanics XML and URDF  
                # is expressed in the COM, so we have only to ensure that the 
                # change the orientation of inertia to match the one of the 
                # simmetryReferenceLink and change the sign of 
                # relevant off diagonal elements (in xz case all the offdiagonal elements related to y)
                # after that we can express the inertia in the frame that we prefer, for example we can leave 
                # it in the simmetryReferenceLink, a long as we set the right pose in the inertial tag
                
                # Get {}^originalLinkInertiaFrame R_simmetryReferenceLink
                originalLinkInertiaFrame_T_simmetryReferenceLink = self.tfman.getHomTransform(originalLink+"CG", "X" + simmetryReferenceLink)
                originalLinkInertiaFrame_R_simmetryReferenceLink = originalLinkInertiaFrame_T_simmetryReferenceLink[0:3,0:3];

                # Get {}^simmetryReferenceLink R_originalLinkInertiaFrame
                simmetryReferenceLink_T_originalLinkInertiaFrame = self.tfman.getHomTransform("X" + simmetryReferenceLink, originalLink+"CG")
                simmetryReferenceLink_R_originalLinkInertiaFrame = simmetryReferenceLink_T_originalLinkInertiaFrame[0:3,0:3];

                # Get  {}^originalLinkInertiaFrame Inertia3D 
                matrix = getlist(originalLinkDict["inertia"])

                units = originalLinkDict['inertiaUnits']
                for i in range(0,len(matrix)):
                    matrix[i] = convert(matrix[i], units)

                originalLinkInertiaFrame_Inertia = numpy.reshape(numpy.array(matrix),[3,3])

                assert(self.isValidInertiaMatrix(originalLinkInertiaFrame_Inertia,1e-3));
                # {}^simmetryReferenceLink Inertia3D = {}^simmetryReferenceLink R_originalLink * {}^originalLink Inertia3D * {}^originalLink R_simmetryReferenceLink
                # sys.stderr.write("simmetryReferenceLink_R_originalLinkInertiaFrame: " + str(simmetryReferenceLink_R_originalLinkInertiaFrame.shape) +"\n");
                #  sys.stderr.write("originalLinkInertiaFrame_Inertia: " + str(originalLinkInertiaFrame_Inertia.shape) +"\n");
                # sys.stderr.write("originalLinkInertiaFrame_R_simmetryReferenceLink: " + str(originalLinkInertiaFrame_R_simmetryReferenceLink.shape) +"\n");
                simmetryReferenceLink_Inertia = numpy.dot(simmetryReferenceLink_R_originalLinkInertiaFrame,numpy.dot(originalLinkInertiaFrame_Inertia,originalLinkInertiaFrame_R_simmetryReferenceLink))


                assert(self.isValidInertiaMatrix(simmetryReferenceLink_Inertia,1e-3)); 

                # xz simmetry: Ixy --> -Ixy , Iyz ---> -Iyz
                simmetryReferenceLink_Inertia[0,1] = -simmetryReferenceLink_Inertia[0,1];
                simmetryReferenceLink_Inertia[1,0] = -simmetryReferenceLink_Inertia[1,0];
                simmetryReferenceLink_Inertia[1,2] = -simmetryReferenceLink_Inertia[1,2];
                simmetryReferenceLink_Inertia[2,1] = -simmetryReferenceLink_Inertia[2,1];

                assert(self.isValidInertiaMatrix(simmetryReferenceLink_Inertia,1e-3)); 

                # The inertia orientation is now the one of R_simmetryReferenceLink, so we have to put in urdf pose  {}^mirroredLink R_simmetryReferenceLink
                (off_dummy,rot) = self.tfman.get("X" + mirroredLink, "X" + simmetryReferenceLink)

                rpy = list(euler_from_quaternion(rot))


                # Save inertia matrix
                # sys.stderr.write("Inertia RPY of link " + str(id) + "is " + str(rpy) + "\n");
                # sys.stderr.write("Inertia matrix of link " + str(id) + "is " + str(simmetryReferenceLink_Inertia) + "\n");
                inertial.inertia = urdf_parser_py.urdf.Inertia()
                inertial.inertia.ixx = simmetryReferenceLink_Inertia[0,0];
                inertial.inertia.ixy = simmetryReferenceLink_Inertia[0,1];
                inertial.inertia.ixz = simmetryReferenceLink_Inertia[0,2];
                inertial.inertia.iyy = simmetryReferenceLink_Inertia[1,1];
                inertial.inertia.iyz = simmetryReferenceLink_Inertia[1,2]
                inertial.inertia.izz = simmetryReferenceLink_Inertia[2,2];
 
                # Save COM and Inertia orientation
                inertial.origin = urdf_parser_py.urdf.Pose(zero(off), zero(rpy))


                # Save also Inertial frame in tfman for consistency
                self.tfman.add(zero(off), rot,"X"+mirroredLink,mirroredLink+"CG")
        else:
                inertial = None

        ### add the link
        link = urdf_parser_py.urdf.Link(id, visual, inertial, collision)


        self.result.add_link(link)



    def getColor(self, s):
        """ Gets a two element list containing a color name,
            and it's rgba. The color selected is based on the mesh name.
            If already seen, returns the saved color
            Otherwise, returns the next available color"""
        if s in self.colormap:
            return self.colormap[s]
        color = COLORS[self.colorindex]
        self.colormap[s] = color
        self.colorindex = (self.colorindex + 1) % len(COLORS)
        return color

    def outputJoint(self, id, parentname):
        """ Outputs URDF for a single joint """
        jointdict = self.joints[id]

        if "Root" in jointdict['name']:
            return

        pid = self.getLinkNameByFrame(jointdict['parent'])
        cid = self.getLinkNameByFrame(jointdict['child'])

        # If the original joint was reversed while building the tree,
        # swap the two ids
        if parentname != pid:
            cid = pid
            pid = parentname

        # Define joint type
        jtype = jointdict['type']

        limits = None
        axis = None

        if 'limits' in jointdict:
            limits = urdf_parser_py.urdf.JointLimit(None, None)
            for (k,v) in jointdict['limits'].items():
                setattr(limits, k, v)
            if( jtype == "continuous" ):
                jtype = "revolute";
        else:
            #if present, load limits from csv joint configuration file
            #note: angle in csv joints configuration file angles are represented as DEGREES
            if( id in self.joint_configuration ):
                conf = self.joint_configuration[id]
                if( ("upper_limit" in conf) or
                   ("lower_limit" in conf) or
                   ("velocity_limit" in conf) or
                   ("effort_limit" in conf) ):
                   limits = urdf_parser_py.urdf.JointLimit()
                   if "upper_limit" in conf:
                       limits.upper = math.radians(float(conf.get("upper_limit")))
                   if "lower_limit" in conf:
                       limits.lower = math.radians(float(conf.get("lower_limit")))
                   if "velocity_limit" in conf:
                       limits.velocity = float(conf.get("velocity_limit"))  
                   else:
                       limits.velocity = self.velocity_limit_fallback                  
                   if "effort_limit" in conf:
                       limits.effort = float(conf.get("effort_limit"))
                   else:
                       limits.effort = self.effort_limit_fallback
                   #if adding limits, switching the joint type to revolute
                   if( jtype == "continuous" ):
                       jtype = "revolute";
            else:
                # if not limits are defined for a prismatic joint, define them
                if( jtype == "prismatic" ):
                    limits = urdf_parser_py.urdf.JointLimit()
                    limits.upper    = 10000
                    limits.lower    = -10000
                    limits.velocity = 10000
                    limits.effort   = 10000

           
                       
        


        # add axis: the axis is expressed in the axisReferenceFrame (normally WORLD)
        #           while in the URDF we have to express it in the child frame
        #           we have then to properly rotate it. 
        if 'axis' in jointdict and jtype != 'fixed':
            axis_string = jointdict['axis'].replace(',', ' ')

            #print("axis string " + str(axis_string))
            axis = [float(axis_el) for axis_el in axis_string.split()]
            #print("axis " + str(axis))
            if( id in self.reverseRotationAxis ):
                for i in range(0,3):
                    axis[i] = -axis[i]

            axis_np = numpy.array(axis)

 
            child_H_axisReferenceFrame = self.tfman.getHomTransform("X"+cid,jointdict['axisReferenceFrame']);

            axis_child = numpy.dot(child_H_axisReferenceFrame[0:3,0:3],axis_np);

            for i in range(0,3):
                axis[i] = axis_child[i];


        # Define the origin
        (off, rot) = self.tfman.get("X" + pid, "X" + cid)
        rpy = list(euler_from_quaternion(rot))
        origin = urdf_parser_py.urdf.Pose(zero(off), zero(rpy))
        
       
        #adding damping and friction (not from simmechanics but from configuration file)
        joint_damping = self.damping_fallback;
        joint_friction = self.friction_fallback;
        if( id in self.joint_configuration ):
            conf = self.joint_configuration[id]
            if "damping" in conf:
                joint_damping = float(conf["damping"])
            if "friction" in conf:
                joint_friction = float(conf["friction"])
        joint_dynamics = urdf_parser_py.urdf.JointDynamics(damping=joint_damping,friction=joint_friction)

        joint = urdf_parser_py.urdf.Joint(id, pid, cid, jtype, limit=limits, axis=axis, origin=origin,dynamics=joint_dynamics)
        self.result.add_joint(joint)

    def getName(self, basename):
        """Return a unique name of the format
           basenameD* where D is the lowest number
           to make the name unique (if the basename is already unique, no number will be added). 
           If a rule for renaming basename is defined in the configuration file, the basename
           will be changed."""
        index = 0
        if basename in self.rename:
           basename = self.rename[basename]
        name = basename
        while name in self.names:
            name = basename + str(index)
            index = index + 1
        self.names[name] = 1
        return name

    def getLinkNameByFrame(self, key):
        """Gets the link name from the frame object"""
        return self.frames[key]['parent']

    def graph(self):
        """For debugging purposes, output a graphviz
           representation of the tree structure, noting
           which joints have been reversed and which have
           been removed"""
        graph = "digraph proe {\n"
        for jkey in self.joints:
            joint = self.joints[jkey]
            pref = joint['parent']
            cref = joint['child']
            #label = pref + ":" + cref
            label = joint['name']
            pkey = self.getLinkNameByFrame(pref)
            ckey = self.getLinkNameByFrame(cref)
            case = 'std'
            if pkey != "GROUND":
                parent = self.links[pkey]
                if not ckey in parent['children']:
                    child = self.links[ckey]
                    if pkey in child['children']:
                        case = 'rev'
                    else:
                        case = 'not'
            pkey = pkey.replace("-", "_")
            ckey = ckey.replace("-", "_")

            if (case == 'std' or case == 'rev') and (joint['type'] != "fixed"):
                style = " penwidth=\"5\""
            else:
                style = "";

            if case == 'std':
                s = pkey + " -> " + ckey + " [ label = \""+label+"\"";
            elif case == 'not':
                s = pkey + " -> " + ckey + " [ label = \""+label+"\" color=\"yellow\""
            elif case == 'rev':
                s = ckey + " -> " + pkey + " [ label = \""+label+"\" color=\"blue\""
            s = s + style + "];"

            if not "Root" in s and "-> SCR_" not in s:
                graph = graph + s + "\n"
        return graph + "}\n"

    def groups(self, root):
        """ For planning purposes, print out lists of
                    all the links between the different joints"""
        self.groups = {}
        self.makeGroup(root, "BASE")
        s = ""
        for key in self.groups.keys():
            s = s + key + ":\n\t"
            ids = self.groups[key]
            for id in ids:
                s = s+id + " "
            s = s + "\n\n"
        return s

    def makeGroup(self, id, gid):
        """ Helper function for recursively gathering
            groups of joints. """
        if gid in self.groups:
            idlist = self.groups[gid]
            idlist.append(id)
        else:
            idlist = [id]
        self.groups[gid] = idlist
        link = self.links[id]
        for child in link['children']:
            jid = link['jointmap'][child]
            joint = self.joints[jid]
            if joint['type'] == 'weld':
                ngid = gid
            else:
                ngid = jid

            self.makeGroup(child, ngid)

def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

>>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
>>> numpy.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
True

"""
    q = numpy.array(quaternion[:4], dtype=numpy.float64, copy=True)
    nq = numpy.dot(q, q)
    if nq < _EPS:
        return numpy.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = numpy.outer(q, q)
    return numpy.array((
        (1.0-q[1, 1]-q[2, 2], q[0, 1]-q[2, 3], q[0, 2]+q[1, 3], 0.0),
        ( q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2], q[1, 2]-q[0, 3], 0.0),
        ( q[0, 2]-q[1, 3], q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        ( 0.0, 0.0, 0.0, 1.0)
        ), dtype=numpy.float64)


def quaternion_from_matrix(matrix):
    """Return quaternion from rotation matrix.

>>> R = rotation_matrix(0.123, (1, 2, 3))
>>> q = quaternion_from_matrix(R)
>>> numpy.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095])
True

"""
    q = numpy.empty((4, ), dtype=numpy.float64)
    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:4, :4]
    t = numpy.trace(M)
    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1, 0] - M[0, 1]
        q[1] = M[0, 2] - M[2, 0]
        q[0] = M[2, 1] - M[1, 2]
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
        q[i] = t
        q[j] = M[i, j] + M[j, i]
        q[k] = M[k, i] + M[i, k]
        q[3] = M[k, j] - M[j, k]
    q *= 0.5 / math.sqrt(t * M[3, 3])
    return q

def getDictionary(tag):
    """Builds a dictionary from the specified xml tag
       where each child of the tag is entered into the dictionary
       with the name of the child tag as the key, and the contents
       as the value. Also removes quotes from quoted values"""
    x = {}
    for child in tag.childNodes:
        if child.nodeType is child.TEXT_NODE:
            continue
        key = str(child.localName)
        if len(child.childNodes) == 1:
            data = str(child.childNodes[0].data)
            if data[0] == '"' and data[-1] == '"':
                if len(data) != 2:
                    x[key] = data[1:-1]
            else:
                x[key] = data
        else:
            data = child.childNodes
            x[key] = data
    return x

def getlist(string):
    """Splits a string of comma delimited floats to
       a list of floats"""
    slist = string.split(",")
    flist = []
    for s in slist:
        flist.append(float(s))
    return flist


def convert(value, units):
    """Convert value from the specified units to mks units"""
    if units == 'kg' or units == 'm' or units == 'kg*m^2':
        return value
    elif units == 'slug*in^2':
        return value * SLUGININ2KGMM
    elif units == 'slug':
        return value * SLUG2KG
    elif units == 'in':
        return value * INCH2METER
    elif units == 'mm':
        return value * MM2M
    else:
        raise Exception("unsupported mass unit: %s" % units)

def matrixToQuaternion(matrix):
    """Concert 3x3 rotation matrix into a quaternion"""
    (R11, R12, R13, R21, R22, R23, R31, R32, R33) = matrix
    # Build 4x4 matrix
    M = [[R11, R21, R31, 0],
         [R12, R22, R32, 0],
         [R13, R23, R33, 0],
         [0,   0,   0,   1]]
    A = numpy.array(M)
    [w,x,y,z] = quaternion_from_matrix(A)
    return [w,x,y,z]

def Invert4x4Matrix(matrix):
    """Invert a 4x4 rototranslation matrix"""
    ret_mat = numpy.identity(4)
    R = matrix[:3,:3]
    p = matrix[:3,3]
    ret_mat[:3,:3] = R.transpose()
    ret_mat[:3,3] = -numpy.dot(R.transpose(),p)
    return ret_mat

def quaternion_to_rpy(quat):
    """Convert quaternion into roll pitch yaw list (in degrees)"""
    rpy = list(euler_from_quaternion(quat))
    for i in range(0, len(rpy)):
        rpy[i] = rpy[i]*180/math.pi
    return rpy

def zero(arr):
    """Converts any numbers less than 1e-7 to 0 in the array"""
    for i in range(0,len(arr)):
        if math.fabs(arr[i]) < 1e-7:
            arr[i] = 0
    return arr


def getMatrix(offset,quaternion):
    """Convert a quaternion + offset to a 4x4 rototranslation matrix"""
    return_matrix = quaternion_matrix(quaternion)
    return_matrix[:3,3] = offset
    return return_matrix

def toGazeboPose(offset,quaternion):
    """Convert an offset + quaternion to a 6x1 Gazebo pose string"""
    rpy = list(euler_from_quaternion(quaternion))
    pose = str(offset[0]) + " " + str(offset[1]) + " " + str(offset[2]) + " " + str(rpy[0]) + " " + str(rpy[1]) + " " + str(rpy[2]);

    return pose;

class CustomTransformManager:
    """Custom class to store several transforms between different frames. 
       The object can then be queried to obtain the transform between two arbitrary frames. """
    def __init__(self):
        self.transform_map = {}

    def add(self,offset,quaternion,parent,child):
        """Store transform for all frames as a list of transform with respect to the world reference frame"""
        # if parent is the world, store the frame directly
        #print("Storing transformation between " + parent + " and " + child)
        #("Quaternion : " +str(quaternion))
        if( parent == WORLD ):
            self.transform_map[child] = getMatrix(offset,quaternion)
            #print(str(self.transform_map[child]))
        elif( child == WORLD ):
            self.transform_map[parent] = Invert4x4Matrix(getMatrix(offset,quaternion))
            #print(str(self.transform_map[parent]))
        else :
            #check if one between parent and child is already part of the manager
            if( self.transform_map.get(parent) is not None ):
                self.transform_map[child] = numpy.dot(self.transform_map[parent],getMatrix(offset,quaternion));
            else: 
                sys.stderr.write("simmechanics_to_urdf: CustomTransformManager: impossible adding a transformation if the parent frame is not already part of the trasforma manager.\n")
                sys.stderr.write("                      Please file an issue at https://github.com/robotology-playground/simmechanics-to-urdf/issues/new .\n");
                assert(False);

    def get(self,parent,child):
        """Return the rototranslation from child to parent ({}^parent T_child) as offset and quaternion"""
        #print("Getting transformation between " + parent + " and " + child)
        return_matrix = self.getHomTransform(parent,child)
        #print(str(return_matrix))
        off = return_matrix[:3,3]
        q = quaternion_from_matrix(return_matrix);

        return [list(off), list(q)]

    def getHomTransform(self,parent,child):
        """Return the homogeneous transformation from child to parent ({}^parent T_child) as 4x4 """
        if( parent == WORLD and child == WORLD ):
            return_matrix = numpy.identity(4)
        elif( parent == WORLD ):
            return_matrix = self.transform_map[child]
        elif( child == WORLD ):
            return_matrix = Invert4x4Matrix(self.transform_map[parent])
        else:
            return_matrix = numpy.dot(Invert4x4Matrix(self.transform_map[parent]),self.transform_map[child]);
        return return_matrix
          

class URDFGazeboSensorsGenerator:
   def __init__(self):
       self.dummy = ""
   
   def getURDFForceTorque(self, jointName, sensorName, directionChildToParent, updateRate=100): 
       gazebo_el = lxml.etree.Element("gazebo" , reference=jointName)
       sensor_el = lxml.etree.SubElement(gazebo_el,"sensor")
       sensor_el.set("name",sensorName);
       always_on_el = lxml.etree.SubElement(sensor_el,"always_on")
       always_on_el.text = str(1);
       update_rate_el = lxml.etree.SubElement(sensor_el,"update_rate")
       update_rate_el.text = str(updateRate);
       sensor_el.set("type","force_torque");
       force_torque_el = lxml.etree.SubElement(sensor_el,"force_torque")
       frame_el = lxml.etree.SubElement(force_torque_el,"frame")
       frame_el.text = "child";  
       measure_direction_el = lxml.etree.SubElement(force_torque_el,"measure_direction")
       if( directionChildToParent ):
           measure_direction_el.text = "child_to_parent"
       else:
           measure_direction_el.text = "parent_to_child"

       return gazebo_el;

   def getURDFIMU(self, linkName, sensorName, pose, updateRate=100):
       #sys.stderr.write("Link name is " + str(linkName) + "\n");
       gazebo_el = lxml.etree.Element("gazebo" , reference=linkName)
       sensor_el = lxml.etree.SubElement(gazebo_el,"sensor")
       sensor_el.set("name",sensorName);
       always_on_el = lxml.etree.SubElement(sensor_el,"always_on")
       always_on_el.text = str(1);
       update_rate_el = lxml.etree.SubElement(sensor_el,"update_rate")
       update_rate_el.text = str(updateRate);
       sensor_el.set("type","imu");
       pose_el = lxml.etree.SubElement(sensor_el,"pose"); 
       pose_el.text = pose;

       return gazebo_el;


def main():
    parser = argparse.ArgumentParser(description='Convert (first generation) SimMechanics XML files to URDF')
    parser.add_argument('filename', nargs='?', help='input SimMechanics (first generation) xml file')
    parser.add_argument('--csv-joints', dest='csv_joints_config', nargs='?', action='store', help='CSV joints configuration file (for options of single joints)')
    parser.add_argument('--yaml', dest='yaml_config', nargs='?', action='store', help='YAML configuration file (for global options)')    
    parser.add_argument('--output', dest='mode', nargs='?', action='store', default='xml', help='output mode, possible options are xml (URDF output, default), graph (DOT output) or none')

    '''
    argc = len(sys.argv)
    if argc == 3:
        filename = sys.argv[1]
        yaml_config = None
        mode = sys.argv[2]
    elif argc == 4:
        filename = sys.argv[1]
        yaml_config = sys.argv[2]
        mode = sys.argv[3]
    else:
        print("Usage: " + sys.argv[0] + "{XML filename} --yaml [yaml_configfile] --csv-joints [csv_joints_configfile] --output {xml|graph|none}")
    '''
    args = parser.parse_args()

    con = Converter()
    con.convert(args.filename, args.yaml_config, args.csv_joints_config, args.mode)



if __name__ == '__main__':
	main()

