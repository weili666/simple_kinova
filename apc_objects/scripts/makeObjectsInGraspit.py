#!/usr/bin/env python
import sys
import os
import lxml.etree as xmlTree

def printHelp(cmd):
    print 'Example: %s example.iv' %cmd
    pass

def makeXml(path,
            iv_file,
            material='plastic',
            useFlockOfBirds='1.0',
            mass= '300',
            cog = '0 0 0',
            inertia_matrix='1 0 0 0 1 0 0 0 1'
            ):
    object = xmlTree.ElementTree()
    root = xmlTree.Element('root')
    object._setroot(root)

    xmlTree.SubElement(root, 'material').text = material
    xmlTree.SubElement(root, 'useFlockOfBirds').text = useFlockOfBirds
    xmlTree.SubElement(root, 'mass').text = mass
    xmlTree.SubElement(root, 'cog').text = cog
    xmlTree.SubElement(root, 'inertia_matrix').text = inertia_matrix
    geometryFile = xmlTree.Element('geometryFile', {'type':'Inventor'})
    geometryFile.text = iv_file+'.iv'
    root.append(geometryFile)

    object.write(path+ '/' + iv_file+'.xml', pretty_print=True)

    pass

def makeObjectInGraspit(path, iv_file):
    makeXml(path, iv_file)
    pass


if __name__=='__main__':
    if len(sys.argv) < 2:
        printHelp(sys.argv[0])
        sys.exit(1)

    iv_file_name = sys.argv[1]
    dirname, basename = os.path.split(iv_file_name)
    iv_file, extention = os.path.splitext(basename)
    extention = extention[1:]

    dest_file = ''

    if extention=='stl' or extention=='STL' :
        dest_file = dirname + '/' + iv_file + '.iv'
        cmd = 'ivcon '+ iv_file_name + '  ' + dest_file
        os.system(cmd)
        makeObjectInGraspit(dirname, iv_file)
        pass
    elif extention=='iv' or extention=='IV' :
        dest_file = iv_file_name
        makeObjectInGraspit(dirname, iv_file)
    else:
        print 'File extention not support, currently support .iv .IV .stl .STL'

    if os.getenv('GRASPIT') == None:
        print "GRASPIT environment variable has not been set"
        sys.exit(1)
    move_xml_cmd = 'mv ' + dirname + '/' + iv_file+'.xml  $GRASPIT/models/objects/'
    os.system(move_xml_cmd )
    mv_iv_cmd = 'mv ' + dest_file + ' $GRASPIT/models/objects/'
    os.system(mv_iv_cmd)


