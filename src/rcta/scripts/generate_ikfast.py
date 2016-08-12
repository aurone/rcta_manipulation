#!/usr/bin/env python

import shutil
import sys
import subprocess
import openravepy
import os

def clean():
    pass

# TODO: create necessary directories
# TODO: migrate include files as well and do manipulations on those

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print 'Usage: {0} <robot.xml file>'.format(sys.argv[0])
        exit(1)

    openravepy.RaveInitialize(True, level = openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging()
    env = openravepy.Environment()
    robot_filename = sys.argv[1]
    robot = env.ReadRobotXMLFile(robot_filename)
    env.Add(robot)

    manipulators = []
    for m in robot.GetManipulators():
        manipulators += [m]
        
    print manipulators
    
    for manip in manipulators:
        robot.SetActiveManipulator(manip)
        ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot, openravepy.IkParameterizationType.Transform6D, freeindices = [manip.GetArmIndices()[4]])
    
    if not ikmodel.load():
        ikmodel.autogenerate()

    #proc = subprocess.Popen('rospack find rcta'.split(), stdout=subprocess.PIPE)
    #robot_filename = proc.stdout.readlines()[0].rstrip() + '/kinbody/hdt.robot.xml'
    
    proc = subprocess.Popen(('openrave.py --database inversekinematics').split() + \
            ['--robot=' + robot_filename] + \
            ('--iktype=transform6d --manipname=hdt_arm --getfilename').split(), stdout=subprocess.PIPE)

    database_name = proc.stdout.readlines()[0]
    print 'Database found in ', database_name
    database_name = database_name.lstrip(' \t\r\n')
    database_name = database_name.rstrip(' \t\r\n')
    print repr(database_name)

    if database_name[-3:len(database_name)] != '.so':
        print 'Shared library not found. Expected {0}, found {1}'.format(
                database_name[0:-3] + '.so', database_name)
        exit(2)

    cpp_file = database_name[0:-3] + '.cpp'
    cpp_file = cpp_file.replace('.x86_64', '')
    print 'Relocating {0} to the \'hdt\' package'.format(cpp_file)

    # move the sources into the hdt_kinematics package
    proc = subprocess.Popen('rospack find rcta'.split(), stdout=subprocess.PIPE)
    dst_dir = '.' #proc.stdout.readlines()[0].rstrip() + '/src'
    shutil.copyfile(cpp_file, dst_dir + '/hdt_arm_transform6d.cpp')
