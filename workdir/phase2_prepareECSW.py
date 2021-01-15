# -*- coding: utf-8 -*-
import sys
import numpy as np

#   STLIB IMPORT
try:
    from splib.animation import AnimationManager , animate
    from stlib.scene.wrapper import Wrapper
    from splib.scenegraph import *

except:
    raise ImportError("ModelOrderReduction plugin depend on SPLIB"\
                     +"Please install it : https://github.com/SofaDefrost/STLIB")

# MOR IMPORT
from mor.utility import sceneCreation as u
from mor.wrapper import replaceAndSave

# Our Phase1 Scene IMPORT
import phase1_snapshots

# Scene parameters
phase = []
phase.append(0)
nbrOfModes = 7
periodSaveGIE = 6
paramWrapper = ('/disk', {'paramForcefield': {'periodSaveGIE': 6, 'prepareECSW': True, 'modesPath': '/home/sofauser/workdir/MOR_test/mor_output/data/modes.txt', 'nbTrainingSet': 25}, 'paramMORMapping': {'input': '@../MechanicalObject', 'modesPath': '/home/sofauser/workdir/MOR_test/mor_output/data/modes.txt'}, 'paramMappedMatrixMapping': {'object1': '@./MechanicalObject', 'object2': '@./MechanicalObject', 'template': 'Vec1d,Vec1d', 'timeInvariantMapping2': True, 'performECSW': False, 'timeInvariantMapping1': True, 'nodeToParse': '@./disk'}})
phaseToSave = [0]

path, param = paramWrapper
param['nbrOfModes'] = 7


def createScene(rootNode):

    # Import Original Scene with the animation added
    # Here we use a wrapper (MORWrapper) that will allow us (with MORreplace)
    # to modify the initial scene and get informations on its structures
    # For more details on the process involved additionnal doc are with :
    #       - mor.wrapper.MORWrapper
    #       - mor.script.sceneCreationUtility

    phase1_snapshots.createScene(Wrapper(rootNode, replaceAndSave.MORreplace, paramWrapper))

    # Add MOR plugin if not found
    u.addPlugin(rootNode,"ModelOrderReduction")

    # Save connectivity list that will allow us after work only on the necessary elements

    if phase == phaseToSave:
        u.saveElements(rootNode,rootNode.dt,replaceAndSave.forcefield)
        param['paramMappedMatrixMapping']['saveReducedMass'] = True

    # Modify the scene to perform hyper-reduction according
    # to the informations collected by the wrapper

    u.modifyGraphScene(rootNode,nbrOfModes,paramWrapper)


    # We Update the link 
    for path , item in replaceAndSave.pathToUpdate.iteritems():
        data , newValue = item
        obj = get(rootNode,path)
        setattr(obj,data,newValue)