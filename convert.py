#!/usr/bin/python

import sys

import xml.dom.minidom
import lxml.etree
import math
import numpy    #for creating matrix to convert to quaternion
import yaml
import urdf_parser_py.urdf

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

        #Start the Custom Transform Manager
        self.tfman = CustomTransformManager()

        # Extra Transforms for Debugging
        self.tfman.add([0,0,0], [0.70682518,0,0,0.70682518], "ROOT", WORLD) # rotate so Z axis is up

    def convert(self, filename, configfile, mode):
        self.mode = mode

        # Parse the configuration file
        self.parseConfig(configfile)

        # Parse the input file
        self.parse(xml.dom.minidom.parse(filename))
        self.buildTree(self.root)

        # Create the output
        self.output(self.root)

        # output the output
        if mode == "xml":
            #print("URDF model to print : \n " + str(self.result) + "\n" )
            print lxml.etree.tostring(self.result.to_xml(),pretty_print=True)
        if mode == "graph":
            print self.graph()
        #if mode == "groups":
        #    print self.groups(root)

    def parseConfig(self, configFile):
        """Parse the Configuration File, if it exists.
           Set the fields the default if the config does
           not set them """
        if configFile == None:
            configuration = {}
        else:
            configuration = yaml.load(file(configFile, 'r'))
            if configuration == None:
                configuration = {}

        self.freezeList = []
        self.redefinedjoints = {}

        self.root = configuration.get('root', None)
        self.extrajoints = configuration.get('extrajoints', {})
        self.filenameformat = configuration.get('filenameformat', "%s")
        self.forcelowercase = configuration.get('forcelowercase', True)
        scale_str = configuration.get('scale', None)
        if( scale_str is not None ):
            self.scale = [float(scale_el) for scale_el in scale_str.split()]
        self.freezeAll = configuration.get('freezeAll', False)
        self.baseframe = configuration.get('baseframe', WORLD)
        self.damping = configuration.get('damping',0.1)
        self.friction = configuration.get('friction',None)
        self.rename = configuration.get('rename',{})

        # Get lists converted to strings
        self.removeList = [ str(e) for e in configuration.get('remove', []) ]
        self.freezeList = [ str(e) for e in configuration.get('freeze', []) ]

        # Get map with key converted to strings
        jointmap = configuration.get('redefinedjoints', {})
        for x in jointmap.keys():
            self.redefinedjoints[str(x)] = jointmap[x]

        # Add Extra Frames
        for frame in configuration.get('moreframes', []):
            self.tfman.add(frame['offset'], frame['orientation'], frame['parent'], frame['child'])


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
            linkdict['color'] = map(float, color.split(",")) + [1.0]

        self.links[uid] = linkdict
        self.parseFrames(frames, uid)

        # Save First Actual Element as Root, if not defined already
        if self.root == None and "geometryFileName" in linkdict:
            self.root = uid

    def parseFrames(self, frames, parent):
        """Parse the frames from xml"""
        for frame in frames:
            if frame.nodeType is frame.TEXT_NODE:
                continue
            fdict = getDictionary(frame)
            fid = str(frame.getAttribute("ref"))
            fdict['parent'] = parent

            offset = getlist(fdict['position'])
            units = fdict['positionUnits']
            for i in range(0, len(offset)):
                offset[i] = convert(offset[i], units)

            orientation = getlist(fdict['orientation'])
            quat = matrixToQuaternion(orientation)
            # If the frame does not have a reference number,
            # use the name plus a suffix (for CG or CS1...
            # otherwise ignore the frame
            if fid == "":
                name = fdict['name']
                if name == "CG":
                    fid = parent + "CG"
                elif name == "CS1":
                    fid = parent + "CS1"
                else:
                    continue

            self.tfman.add(offset, quat, WORLD, fid)
            self.frames[fid] = fdict

    def parseJoint(self, element):
        """Parse the joint from xml"""
        dict = getDictionary(element)
        joint = {}
        joint['name'] = dict['name']
        uid = self.getName(joint['name'])

        frames = element.getElementsByTagName("Frame")
        joint['parent'] = str(frames[0].getAttribute("ref"))
        joint['child'] = str(frames[1].getAttribute("ref"))
        type = element.getElementsByTagName("Primitive")

        # If there multiple elements, assume a fixed joint
        if len(type)==1:
            pdict = getDictionary(type[0])
            joint['type'] = pdict['name']
            joint['axis'] = pdict['axis']
            if joint['type'] == 'weld':
                joint['type'] = 'fixed'
        else:
            joint['type'] = 'fixed'

        # Ignore joints on the remove list
        if joint['parent'] in self.removeList:
            return

        # Force joints to be fixed on the freezeList
        if joint['parent'] in self.freezeList or self.freezeAll:
            joint['type'] = 'fixed'

        # Redefine specified joints on redefined list
        if joint['parent'] in self.redefinedjoints.keys():
            jdict = self.redefinedjoints[joint['parent']]
            if 'name' in jdict:
                uid = jdict['name']

            # Default to continuous joints
            joint['type'] = jdict.get('type', 'continuous')

            if 'axis' in jdict:
                print("axis" + str(jdict['axis']))
                joint['axis'] = jdict['axis']
            if 'limits' in jdict:
                joint['limits'] = jdict['limits']

        #if the joint is revolute but no limits are defined, switch to continuous
        if 'limits' not in joint.keys()  and joint['type'] == "revolute":
            joint['type'] = "continuous";


        self.joints[uid] = joint

    def buildTree(self, root):
        """Reduce the graph structure of links and joints to a tree
           by breadth first search. Then construct new coordinate frames
           from new tree structure"""

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

        # build new coordinate frames
        for id in self.links:
            link = self.links[id]
            if not 'parent' in link:
                continue
            parentid = link['parent']
            if parentid == "GROUND":
                ref = self.baseframe
            else:
                joint = self.joints[link['jointmap'][parentid]]
                ref = joint['parent']
            # The root of each link is the offset to the joint
            # and the rotation of the CS1 frame
            (off1, rot1) = self.tfman.get(WORLD, ref)
            (off2, rot2) = self.tfman.get(WORLD, id + "CS1")
            self.tfman.add(off1, rot2, WORLD, "X" + id)


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

    def outputLink(self, id):
        """ Creates the URDF output for a single link """

        linkdict = self.links[id]
        if linkdict['name'] == "RootPart":
            return

        visual = urdf_parser_py.urdf.Visual()
        inertial = urdf_parser_py.urdf.Inertial()
        collision = urdf_parser_py.urdf.Collision()

        # Define Geometry
        filename = linkdict['geometryFileName']
        if self.forcelowercase:
            filename = filename.lower()
        filename = self.filenameformat % filename

        visual.geometry = urdf_parser_py.urdf.Mesh(filename, self.scale)
        collision.geometry = visual.geometry

        # Define Inertial Frame
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
        rpy = list(euler_from_quaternion(rot))
        inertial.origin = urdf_parser_py.urdf.Pose(zero(off), zero(rpy))

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


        # Define the parent and child
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

        if 'axis' in jointdict and jtype != 'fixed':
            axis_string = jointdict['axis'].replace(',', ' ')

        # Define the origin
        (off, rot) = self.tfman.get("X" + pid, "X" + cid)
        rpy = list(euler_from_quaternion(rot))
        origin = urdf_parser_py.urdf.Pose(zero(off), zero(rpy))
        
        #print("axis string " + str(axis_string))
        axis = [float(axis_el) for axis_el in axis_string.split()]
        #print("axis " + str(axis))

        #adding damping and friction (not from simmechanics but from configuration file)
        joint_dynamics = urdf_parser_py.urdf.JointDynamics(damping=self.damping,friction=self.friction)

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
            label = pref + ":" + cref
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

class CustomTransformManager:
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
            print("Not implemented");

    def get(self,parent,child):
        """"""
        #print("Getting transformation between " + parent + " and " + child)
        if( parent == WORLD and child == WORLD ):
            return_matrix = numpy.identity(4)
        elif( parent == WORLD ):
            return_matrix = self.transform_map[child]
        elif( child == WORLD ):
            return_matrix = Invert4x4Matrix(self.transform_map[parent])
        else:
            return_matrix = numpy.dot(Invert4x4Matrix(self.transform_map[parent]),self.transform_map[child]);
        #print(str(return_matrix))
        off = return_matrix[:3,3]
        q = quaternion_from_matrix(return_matrix);

        return [list(off), list(q)]

if __name__ == '__main__':
    argc = len(sys.argv)
    if argc == 3:
        filename = sys.argv[1]
        config = None
        mode = sys.argv[2]
    elif argc == 4:
        filename = sys.argv[1]
        config = sys.argv[2]
        mode = sys.argv[3]
    else:
        print "Usage: " + sys.argv[0] + " {XML filename} [configfile] {xml|graph|none}"
        sys.exit(-1)
    con = Converter()
    con.convert(filename, config, mode)


