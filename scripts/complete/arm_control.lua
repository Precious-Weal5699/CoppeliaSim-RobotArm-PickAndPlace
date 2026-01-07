sim = require'sim'
simIK = require'simIK'

-- ========== GLOBAL CONFIG ==========
isPicking = false
isLoadAreaFull = false

-- Load area grid configuration
loadGrid = {
    originHandle = '/load_area',   -- name/path of your load area shape or dummy
    rows = 4,                      -- number of rows
    cols = 5,                      -- number of columns
    spacingX = 0.12,               -- X spacing between boxes (adjust for box width)
    spacingY = 0.2,               -- Y spacing between boxes (adjust for box depth)
    startOffset = { -0.2, -0.3, 0 }, -- offset from load_area origin to first cell
    boxHeight = 0.1,               -- height of one box (for stacking properly)
}
currentLoadIndex = 0  -- tracks which grid cell to fill next

-- ========== HELPER FUNCTIONS ==========

function hopThroughConfigs(path, joints, reverse, dynModel)
    local lb = sim.setStepping(true)
    local s = 1
    local g = #path / 6
    local incr = 1
    if reverse then
        s = #path / 6
        g = 1
        incr = -1
    end
    for i = s, g, incr do
        if dynModel then
            for j = 1, #joints, 1 do
                sim.setJointTargetPosition(joints[j], path[(i - 1) * 6 + j])
            end
        else
            for j = 1, #joints, 1 do
                sim.setJointPosition(joints[j], path[(i - 1) * 6 + j])
            end
        end
        sim.step()
    end
    sim.setStepping(lb)
end

-- Computes the world position/orientation of the next grid cell in the load area
function getNextLoadPose()
    local loadAreaHandle = sim.getObject(loadGrid.originHandle)
    local loadAreaMatrix = sim.getObjectMatrix(loadAreaHandle, -1)

    local idx = currentLoadIndex
    local col = math.floor(idx % loadGrid.cols)
    local row = math.floor(idx / loadGrid.cols)

    local offsetLocal = {
        loadGrid.startOffset[1] + col * loadGrid.spacingX,
        loadGrid.startOffset[2] + row * loadGrid.spacingY,
        loadGrid.boxHeight / 2 + 0.02
    }
    
    
    local worldPos = sim.multiplyVector(loadAreaMatrix, offsetLocal)

    return worldPos, {0, math.rad(-90), 0}
end

-- ========== MAIN PICK & PLACE FUNCTION ==========
function handleBoxPickAndPlace()
    local boxHandle = sim.getInt32Signal('detected_box')
    if not boxHandle or boxHandle == -1 then
        print('Robot: no valid box handle detected, using fallback container_1')
        boxHandle = sim.getObject('/container_1')
    else
        print('Robot: box ready to pick, handle =', boxHandle)
    end

    -- Robot is busy
    isPicking = true
    sim.setInt32Signal('robot_picking', 1)

    -- Get robot references
    local simBase = sim.getObject('..')
    local simTip = sim.getObject('../BarrettHand/tip')
    local gripperAttachPoint = sim.getObject('/UR10/BarrettHand/attachPoint')

    -- Get robot joints
    local simJoints = {}
    for i = 1, 6, 1 do
        simJoints[i] = sim.getObject('../joint', {index = i - 1})
    end

    local dynModel = sim.isDynamicallyEnabled(simJoints[1])
    local handScriptHandle = sim.getObject('/UR10/BarrettHand/Script')

    -- Record initial joint positions
    local initialJointPositions = {}
    for i = 1, #simJoints, 1 do
        initialJointPositions[i] = sim.getJointPosition(simJoints[i])
    end

    -- ======= PICK TARGET =======
    local simPick = sim.createDummy(0.02)
    -- use the real stopped box position (if provided)
    local bx = sim.getFloatSignal('box_pos_x') or 0
    local by = sim.getFloatSignal('box_pos_y') or 0
    local bz = sim.getFloatSignal('box_pos_z') or 0.05
    sim.setObjectPosition(simPick, -1, {bx, by, bz + 0.05})
    sim.setObjectOrientation(simPick, boxHandle, {0, math.rad(-90), 0})

    -- ======= PLACE TARGET =======
    local placePos, placeOri = getNextLoadPose()
    local simPlace = sim.createDummy(0.02)
    sim.setObjectPosition(simPlace, -1, placePos)
    sim.setObjectOrientation(simPlace, -1, placeOri)

    -- ======= CREATE IK PATHS =======
    local ikEnvPick = simIK.createEnvironment()
    local ikGroupPick = simIK.createGroup(ikEnvPick)
    local _, mapPick = simIK.addElementFromScene(ikEnvPick, ikGroupPick, simBase, simTip, simPick, simIK.constraint_pose)
    local ikTipPick = mapPick[simTip]
    local ikJointsPick = {}
    for i = 1, #simJoints, 1 do
        ikJointsPick[i] = mapPick[simJoints[i]]
    end
    local pickPath = simIK.generatePath(ikEnvPick, ikGroupPick, ikJointsPick, ikTipPick, 80)
    simIK.eraseEnvironment(ikEnvPick)
    

    local ikEnvPlace = simIK.createEnvironment()
    local ikGroupPlace = simIK.createGroup(ikEnvPlace)
    local _, mapPlace = simIK.addElementFromScene(ikEnvPlace, ikGroupPlace, simBase, simTip, simPlace, simIK.constraint_pose)
    local ikTipPlace = mapPlace[simTip]
    local ikJointsPlace = {}
    for i = 1, #simJoints, 1 do
        ikJointsPlace[i] = mapPlace[simJoints[i]]
    end
    local placePath = simIK.generatePath(ikEnvPlace, ikGroupPlace, ikJointsPlace, ikTipPlace, 80)
    simIK.eraseEnvironment(ikEnvPlace)

    -- ======= EXECUTE PICK & PLACE =======
    hopThroughConfigs(pickPath, simJoints, false, dynModel)
    sim.wait(0.5)
    sim.callScriptFunction('closeHand', handScriptHandle)
    sim.wait(1.0)
    sim.setObjectParent(boxHandle, gripperAttachPoint, true)
    sim.wait(1.0)
    sim.removeObjects({simPick})

    -- Move to place
    hopThroughConfigs(placePath, simJoints, false, dynModel)
    sim.wait(0.2)
    -- Release box and retreat slightly before opening
    sim.setObjectParent(boxHandle, -1, true)
    sim.removeObjects({simPlace})
    
    sim.callScriptFunction('openHand', handScriptHandle)
    local partialOpenTime = 0.05 -- (Try 0.1, 0.2, 0.3, etc.)
    sim.wait(partialOpenTime)
    --sim.wait(0.05)--
    
    -- 3. Tell hand to FREEZE in its new, "cracked open" position
    sim.callScriptFunction('stopHand', handScriptHandle)
    sim.wait(0.05) -- Wait for it to settle

    -- LIFT UP to avoid hitting load stack (inserted code)
    local liftAmount = 0.08 -- meters; tune as needed
    local tipPos = sim.getObjectPosition(simTip, -1)
    local tipOri = sim.getObjectOrientation(simTip, -1)
    local liftDummy = sim.createDummy(0.02)
    sim.setObjectPosition(liftDummy, -1, {tipPos[1], tipPos[2], tipPos[3] + liftAmount})
    sim.setObjectOrientation(liftDummy, -1, tipOri)

    local ikEnvLift = simIK.createEnvironment()
    local ikGroupLift = simIK.createGroup(ikEnvLift)
    local _, mapLift = simIK.addElementFromScene(ikEnvLift, ikGroupLift, simBase, simTip, liftDummy, simIK.constraint_pose)
    local ikTipLift = mapLift[simTip]
    local ikJointsLift = {}
    for i=1,#simJoints,1 do
        ikJointsLift[i] = mapLift[simJoints[i]]
    end
    local liftPath = simIK.generatePath(ikEnvLift, ikGroupLift, ikJointsLift, ikTipLift, 80)
    simIK.eraseEnvironment(ikEnvLift)
    if liftPath and #liftPath>0 then
        hopThroughConfigs(liftPath, simJoints, false, dynModel)
    end
    sim.removeObjects({liftDummy})
    sim.wait(0.05)
    
    -- 5. NOW that we are clear, open the hand completely
    sim.callScriptFunction('openHand', handScriptHandle)
    sim.wait(0.5)

    -- Increment load index (row-by-row placement)
    currentLoadIndex = currentLoadIndex + 1
    
    if currentLoadIndex >= (loadGrid.rows * loadGrid.cols) then
        isLoadAreaFull = true -- Set the robot's internal flag
        sim.setInt32Signal('load_area_full', 1) -- Set the signal for the conveyor
        sim.addStatusbarMessage('!!! LOAD AREA FULL. Robot stopping. !!!')
        print('!!! LOAD AREA FULL. Robot stopping. !!!')
    end

    -- Signal conveyor to resume
    sim.clearInt32Signal('box_ready')
    sim.clearInt32Signal('detected_box')
    isPicking = false
    sim.clearInt32Signal('robot_picking')
    sim.setInt32Signal('resume_conveyor', 1)

    -- Return to initial pose
    sim.wait(0.5)
    for i = 1, #simJoints, 1 do
        sim.setJointTargetPosition(simJoints[i], initialJointPositions[i])
    end
    for step = 1, 100, 1 do
        sim.step()
    end
end

-- ========== MAIN LOOP ==========
function sysCall_thread()
    local simJoints = {}
    for i = 1, 6, 1 do
        simJoints[i] = sim.getObject('../joint', {index = i - 1})
    end
    sim.step()

    -- Hold initial pose at startup
    local initialJointPositions = {}
    for i = 1, #simJoints, 1 do
        initialJointPositions[i] = sim.getJointPosition(simJoints[i])
        sim.setJointTargetPosition(simJoints[i], initialJointPositions[i])
    end
    for k = 1, 50 do sim.step() end
    print('Robot initial pose held.')

    -- Main loop
    while sim.getSimulationState() ~= sim.simulation_advancing_abouttostop do
        while sim.getInt32Signal('box_ready') ~= 1 do
            sim.step()
        end
        
        -- !!! --- NEW CHECK --- !!!
        if isLoadAreaFull then
            -- If we are full, don't pick.
            print("Load area is full. Ignoring new box.")
            sim.clearInt32Signal('box_ready')
            sim.clearInt32Signal('detected_box')
        else
            -- We are not full, so run the pick.
            handleBoxPickAndPlace()
        end
        sim.wait(0.2)
    end
end
