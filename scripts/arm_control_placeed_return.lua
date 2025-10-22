sim=require'sim'
simIK=require'simIK'

function hopThroughConfigs(path,joints,reverse,dynModel)
    local lb=sim.setStepping(true)
    local s=1
    local g=#path/6
    local incr=1
    if reverse then
        s=#path/6
        g=1
        incr=-1
    end
    
    for i=s,g,incr do
        if dynModel then
            for j=1,#joints,1 do
                sim.setJointTargetPosition(joints[j],path[(i-1)*6+j])
            end
        else
            for j=1,#joints,1 do
                sim.setJointPosition(joints[j],path[(i-1)*6+j])
            end
        end
        sim.step()
    end
    sim.setStepping(lb)
end

function sysCall_thread()
    local simBase=sim.getObject('..')
    local simTip=sim.getObject('../BarrettHand/tip')
    --goalpose for one path
    --local simGoal=sim.getObject('/container_1/goalPose')
    
    --temporarily attach box to gripper
    local boxHandle = sim.getObject('/container_1')
    local gripperAttachPoint = sim.getObject('/UR10/BarrettHand/attachPoint')
    
    -- Get all required goal handles
    local simPick = sim.getObject('/container_1/pickPose')
    local simPlace = sim.getObject('/placePose')
    
    local simJoints={}
    for i=1,6,1 do
        simJoints[i]=sim.getObject('../joint',{index=i-1})
    end
    sim.step() -- make sure we have skipped the first simulation step, 
                       -- otherwise following cmd won't reflect reality
                       
    -- ? Record initial joint positions before motion
    local initialJointPositions = {}
    for i=1,#simJoints,1 do
        initialJointPositions[i] = sim.getJointPosition(simJoints[i])
    end
    
    local dynModel=sim.isDynamicallyEnabled(simJoints[1])
    
    -- Get handle to the BarrettHand script once:
    local handScriptHandle = sim.getObject('/UR10/BarrettHand/Script')


    -- Prepare an ik group, using the convenience function 'simIK.addElementFromScene':
    local ikEnv=simIK.createEnvironment()
    
    -- ========== PICK PATH ==========
    local ikEnvPick = simIK.createEnvironment()
    local ikGroupPick = simIK.createGroup(ikEnvPick)
    local _, simToIkMapPick = simIK.addElementFromScene(ikEnvPick, ikGroupPick, simBase, simTip, simPick, simIK.constraint_pose)
    local ikTipPick = simToIkMapPick[simTip]
    local ikJointsPick = {}
    for i=1,#simJoints,1 do
        ikJointsPick[i] = simToIkMapPick[simJoints[i]]
    end
    local pickPath = simIK.generatePath(ikEnvPick, ikGroupPick, ikJointsPick, ikTipPick, 300)
    simIK.eraseEnvironment(ikEnvPick) -- clean up immediately

    -- ========== PLACE PATH ==========
    local ikEnvPlace = simIK.createEnvironment()
    local ikGroupPlace = simIK.createGroup(ikEnvPlace)
    local _, simToIkMapPlace = simIK.addElementFromScene(ikEnvPlace, ikGroupPlace, simBase, simTip, simPlace, simIK.constraint_pose)
    local ikTipPlace = simToIkMapPlace[simTip]
    local ikJointsPlace = {}
    for i=1,#simJoints,1 do
        ikJointsPlace[i] = simToIkMapPlace[simJoints[i]]
    end
    local placePath = simIK.generatePath(ikEnvPlace, ikGroupPlace, ikJointsPlace, ikTipPlace, 300)
    simIK.eraseEnvironment(ikEnvPlace)
    
     -- ===== EXECUTE PATHS =====
    -- Move to pick
    hopThroughConfigs(pickPath, simJoints, false, dynModel)
    sim.wait(0.5)
    sim.callScriptFunction('closeHand', handScriptHandle)
    sim.wait(1.5)
    sim.setObjectParent(boxHandle, gripperAttachPoint, true)
    
    -- Move to place
    hopThroughConfigs(placePath, simJoints, false, dynModel)
    sim.wait(0.5)
    sim.setObjectParent(boxHandle, -1, true)
    sim.callScriptFunction('openHand', handScriptHandle)
    sim.wait(1.0)
    
    -- Return to initial pose
    sim.wait(0.5)
    for i=1,#simJoints,1 do
        sim.setJointTargetPosition(simJoints[i], initialJointPositions[i])
    end

    -- Step a few times to make sure it moves fully back
    for step=1,100,1 do
        sim.step()
    end
    
end