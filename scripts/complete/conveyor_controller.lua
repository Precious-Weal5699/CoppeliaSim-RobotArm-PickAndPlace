sim = require'sim'

function sysCall_init()
    -- Get handles
    conveyorHandle = sim.getObjectHandle('/conveyorSystem')
    spawnPoint = sim.getObjectHandle('/conveyorSystem/spawnPoint')
    prototypeBox = sim.getObjectHandle('/container_1')
    sensor = sim.getObjectHandle('/conveyorSystem/proximitySensor')
    spawnSensor = sim.getObjectHandle('/conveyorSystem/spawnPoint/spawnSensor')

    -- Parameters
    spawnInterval = 6
    lastSpawnTime = sim.getSimulationTime()
    conveyorSpeed = 0.15
    conveyorRunning = true

    detectionTime = nil
    detectedObjectHandle = -1
    lastDetectedHandle = -1
    boxHasStopped = false
    
end


function sysCall_actuation()
    local currentTime = sim.getSimulationTime()

    -- !!! --- NEW CHECK for FULL LOAD AREA --- !!!
    if sim.getInt32Signal('load_area_full') == 1 then
        if conveyorRunning then
            print("Conveyor stopping: Load area is full.")
            stopConveyor()
        end
        -- Clear any lingering 'ready' signals
        sim.clearInt32Signal('box_ready') 
        return -- Halt all conveyor activity
    end
    
    -- 1?? If robot is busy, keep conveyor stopped
    if sim.getInt32Signal('robot_picking') == 1 then
        if conveyorRunning then
            stopConveyor()
        end
        return
    end

    -- 2?? Spawn new boxes
    if conveyorRunning and (currentTime - lastSpawnTime > spawnInterval) then
        spawnBox()
        lastSpawnTime = currentTime
    end

    -- 3?? Detect box and stop conveyor
    local result, _, _, detectedHandle = sim.checkProximitySensor(sensor, sim.handle_all)
    if result > 0 and conveyorRunning then
        stopConveyor()
        detectionTime = currentTime
        lastDetectedHandle = detectedHandle
        boxHasStopped = false
    end

    -- 4?? Wait until conveyor actually stops (box velocity near zero)
    if detectionTime and not conveyorRunning then
        if sim.isHandle(lastDetectedHandle) then
            local vel, _ = sim.getObjectVelocity(lastDetectedHandle)
            local speed = math.sqrt(vel[1]^2 + vel[2]^2 + vel[3]^2)

            if speed < 0.01 then  -- ? box fully stopped (<1 cm/s)
                if not boxHasStopped then
                    print("Box has fully stopped, notifying robot...")
                    sim.setInt32Signal('box_ready', 1)
                    sim.setInt32Signal('detected_box', lastDetectedHandle)

                    local pos = sim.getObjectPosition(lastDetectedHandle, -1)
                    sim.setFloatSignal('box_pos_x', pos[1])
                    sim.setFloatSignal('box_pos_y', pos[2])
                    sim.setFloatSignal('box_pos_z', pos[3])

                    boxHasStopped = true
                    detectionTime = nil
                end
            end
        end
    end

    -- 5?? Resume conveyor after robot finishes
    if sim.getInt32Signal('resume_conveyor') == 1 then
        startConveyor()
        sim.clearInt32Signal('resume_conveyor')
    end
end


-- ?? Helper functions
function stopConveyor()
    sim.setBufferProperty(conveyorHandle, 'customData.__ctrl__', sim.packTable({vel = 0.0}))
    conveyorRunning = false
end

function startConveyor()
    -- Stop the conveyor completely
    --sim.setBufferProperty(conveyorHandle, 'customData.__ctrl__', sim.packTable({vel = 0}))
    
    -- Restart with velocity
    sim.setBufferProperty(conveyorHandle, 'customData.__ctrl__', sim.packTable({vel = conveyorSpeed}))
    conveyorRunning = true
    
    -- Force boxes to "re-land" on conveyor (break and remake contact)
    local allShapes = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
    for i=1, #allShapes do
        local alias = sim.getObjectAlias(allShapes[i])
        if alias and alias:find('container') then
            -- Lift box very slightly to break conveyor contact
            local pos = sim.getObjectPosition(allShapes[i], -1)
            pos[3] = pos[3] + 0.009  -- 2mm up
            sim.setObjectPosition(allShapes[i], -1, pos)
            
            -- Reset its dynamics
            sim.resetDynamicObject(allShapes[i])
        end
    end
end

function spawnBox()
    -- Check if spawn area is clear before spawning
    local result, _, _, detectedHandle = sim.checkProximitySensor(spawnSensor, sim.handle_all)
    if result > 0 then
        -- Something is under the spawn area, skip spawning this cycle
        print("Spawn skipped: area occupied by box handle ", detectedHandle)
        return
    end
    
    -- If clear, spawn a new box
    local newBox = sim.copyPasteObjects({prototypeBox}, 0)[1]
    
    local prototypeAlias = sim.getObjectAlias(prototypeBox)
    sim.setObjectAlias(newBox, prototypeAlias)
    
    local beltPos = sim.getObjectPosition(spawnPoint, -1)
    local pos = {beltPos[1], beltPos[2], beltPos[3] + 0.15}
    sim.setObjectPosition(newBox, -1, pos)
    sim.setObjectInt32Parameter(newBox, sim.shapeintparam_static, 0)
    startConveyor()
end

