sim=require'sim'
simUI=require'simUI'

-- See the end of the script for instructions on how to do efficient grasping

-- ============ FUNCTION DEFINITIONS (callable externally) ============
function closeHand()
    closing = true
    attachedShape = nil

    local index = 0
    while true do
        local shape = sim.getObjects(index, sim.sceneobject_shape)
        if shape == -1 then break end
        if (sim.getObjectInt32Param(shape, sim.shapeintparam_static) == 0) and
           (sim.getObjectInt32Param(shape, sim.shapeintparam_respondable) ~= 0) and
           (sim.checkProximitySensor(objectSensor, shape) == 1) then
            attachedShape = shape
            sim.setObjectParent(attachedShape, connector, true)
            print('Attached shape:', sim.getObjectAlias(attachedShape, 1))
            break
        end
        index = index + 1
    end
end

function sysCall_init() 
    jointHandles={{-1,-1,-1},{-1,-1,-1},{-1,-1,-1}}
    firstPartTorqueSensorHandles={-1,-1,-1}
    for i=0,2,1 do
        if (i~=1) then
            jointHandles[i+1][1]=sim.getObject('../jointA_'..i)
        end
        jointHandles[i+1][2]=sim.getObject('../jointB_'..i)
        jointHandles[i+1][3]=sim.getObject('../jointC_'..i)
        firstPartTorqueSensorHandles[i+1]=sim.getObject('../jointB_'..i)
    end
    modelHandle=sim.getObject('..')
    closing=false
    sliderV=50
    firstPartLocked={false,false,false}
    needFullOpening={0,0,0}
    firstPartTorqueOvershootCount={0,0,0}
    firstPartTorqueOvershootCountRequired=1
    firstPartMaxTorque=2.5

    closingVel=60*math.pi/180
    openingVel=-120*math.pi/180

    closingOpeningTorque=10

    for i=1,3,1 do
        sim.setObjectInt32Param(jointHandles[i][2],sim.jointintparam_motor_enabled,1)
        sim.setObjectInt32Param(jointHandles[i][2],sim.jointintparam_ctrl_enabled,0)
        sim.setObjectInt32Param(jointHandles[i][3],sim.jointintparam_motor_enabled,1)
        sim.setObjectInt32Param(jointHandles[i][3],sim.jointintparam_ctrl_enabled,1)
        sim.setJointTargetForce(jointHandles[i][2],closingOpeningTorque)
        sim.setJointTargetForce(jointHandles[i][3],closingOpeningTorque)
        sim.setJointTargetVelocity(jointHandles[i][2],-closingVel)
        sim.setJointTargetVelocity(jointHandles[i][3],-closingVel/3)
    end
    -- *** Force-lock the spread joints (jointA) to their current pose so they don't "spread" ***
    for i=1,3,1 do
        local ja = jointHandles[i][1]
        if ja and ja~=-1 then
            sim.setObjectInt32Param(ja, sim.jointintparam_motor_enabled, 1)
            -- enable position (ctrl) mode so we can hold current angle
            sim.setObjectInt32Param(ja, sim.jointintparam_ctrl_enabled, 1)
            -- set a strong holding force so external motions won't easily move them
            sim.setJointTargetForce(ja, closingOpeningTorque*100)
            -- set target position to current position (locks the joint where it already is)
            sim.setJointTargetPosition(ja, sim.getJointPosition(ja))
        end
    end
    
    --New line--
    connector = sim.getObject('../attachPoint')
    objectSensor = sim.getObject('../attachProxSensor')
    attachedShape = nil
    --New Line --
end

function sysCall_cleanup() 
    for i=1,3,1 do
        sim.setObjectInt32Param(jointHandles[i][2],sim.jointintparam_motor_enabled,1)
        sim.setObjectInt32Param(jointHandles[i][2],sim.jointintparam_ctrl_enabled,0)
        sim.setObjectInt32Param(jointHandles[i][3],sim.jointintparam_motor_enabled,1)
        sim.setObjectInt32Param(jointHandles[i][3],sim.jointintparam_ctrl_enabled,1)
        sim.setJointTargetForce(jointHandles[i][2],closingOpeningTorque)
        sim.setJointTargetForce(jointHandles[i][3],closingOpeningTorque)
        sim.setJointTargetVelocity(jointHandles[i][2],-closingVel)
        sim.setJointTargetVelocity(jointHandles[i][3],-closingVel/3)
    end
end 

function sysCall_sensing()
    local s=sim.getObjectSel()
    local show=(s and #s==1 and s[1]==modelHandle)
    if show then
        if not ui then
            local xml =[[<ui title="xxxx" closeable="false" placement="relative" layout="form">
                    <button id="1" text="open" checkable="true" checked="true" auto-exclusive="true" on-click="openClicked"/>
                    <button id="2" text="close" checkable="true" auto-exclusive="true" on-click="closeClicked"/>
                    
                    <label text="Finger angle"/>
                    <hslider id="3" on-change="fingerAngleMoved"/>
            </ui>]]
            ui=simUI.create(xml)
            if uiPos then
                simUI.setPosition(ui,uiPos[1],uiPos[2])
            else
                uiPos={}
                uiPos[1],uiPos[2]=simUI.getPosition(ui)
            end
            simUI.setTitle(ui,sim.getObjectAlias(modelHandle,5))
            simUI.setButtonPressed(ui,1,not closing)
            simUI.setButtonPressed(ui,2,closing)
            simUI.setSliderValue(ui,3,sliderV)
        end
    else
        if ui then
            uiPos[1],uiPos[2]=simUI.getPosition(ui)
            simUI.destroy(ui)
            ui=nil
        end
    end
end

function openClicked(ui,id)
    closing=false
end

function closeClicked(ui,id)
    closing=true
end

function fingerAngleMoved(ui,id,v)
    sliderV=v
    sim.setJointTargetPosition(jointHandles[1][1],-math.pi*0.5+math.pi*sliderV/100)
    sim.setJointTargetPosition(jointHandles[3][1],-math.pi*0.5+math.pi*sliderV/100)
end

function sysCall_actuation()
    for i=1,3,1 do
        if (closing)and(needFullOpening[1]~=2)and(needFullOpening[2]~=2)and(needFullOpening[3]~=2) then
            if (firstPartLocked[i]) then
                sim.setJointTargetVelocity(jointHandles[i][3],closingVel/3)
            else
                t=simJointGetForce(firstPartTorqueSensorHandles[i])
                if (t)and(t<-firstPartMaxTorque) then
                    firstPartTorqueOvershootCount[i]=firstPartTorqueOvershootCount[i]+1
                else
                    firstPartTorqueOvershootCount[i]=0
                end
                if (firstPartTorqueOvershootCount[i]>=firstPartTorqueOvershootCountRequired) then
                    needFullOpening[i]=1
                    firstPartLocked[i]=true
                    -- First joint is now locked and holding the position:
                    sim.setObjectInt32Param(jointHandles[i][2],sim.jointintparam_ctrl_enabled,1)
                    sim.setJointTargetForce(jointHandles[i][2],closingOpeningTorque*100)
                    sim.setJointTargetPosition(jointHandles[i][2],sim.getJointPosition(jointHandles[i][2]))
                    -- second joint is now not in position control anymore:
                    sim.setObjectInt32Param(jointHandles[i][3],sim.jointintparam_ctrl_enabled,0)
                    sim.setJointTargetVelocity(jointHandles[i][3],closingVel/3)
                else
                    sim.setJointTargetVelocity(jointHandles[i][2],closingVel)
                    sim.setJointTargetPosition(jointHandles[i][3],(45*math.pi/180)+sim.getJointPosition(jointHandles[i][2])/3)
                end
            end
        else
            if (needFullOpening[i]==1) then
                needFullOpening[i]=2
            end
            sim.setJointTargetVelocity(jointHandles[i][3],openingVel/3)
            if (firstPartLocked[i]) then
                jv=sim.getJointPosition(jointHandles[i][3])
                if (jv<45.5*math.pi/180) then
                    firstPartLocked[i]=false -- we unlock the first part
                    sim.setObjectInt32Param(jointHandles[i][2],sim.jointintparam_ctrl_enabled,0)
                    sim.setJointTargetForce(jointHandles[i][2],closingOpeningTorque)
                    sim.setJointTargetVelocity(jointHandles[i][2],openingVel)
                end
            else
                if (needFullOpening[i]~=0) then
                    jv3=sim.getJointPosition(jointHandles[i][3])
                    jv2=sim.getJointPosition(jointHandles[i][2])
                    if (jv3<45.5*math.pi/180)and(jv2<2*math.pi/180) then
                        needFullOpening[i]=0
                        -- second joint is now again in position control:
                        sim.setObjectInt32Param(jointHandles[i][3],sim.jointintparam_ctrl_enabled,1)
                        sim.setJointTargetPosition(jointHandles[i][3],(45*math.pi/180)+sim.getJointPosition(jointHandles[i][2])/3)
                    end
                else
                    sim.setJointTargetVelocity(jointHandles[i][2],openingVel)
                    sim.setJointTargetPosition(jointHandles[i][3],(45*math.pi/180)+sim.getJointPosition(jointHandles[i][2])/3)
                end
            end
        end
    end

    
    
    -- You have basically 2 alternatives to grasp an object:
    --
    -- 1. You try to grasp it in a realistic way. This is quite delicate and sometimes requires
    --    to carefully adjust several parameters (e.g. motor forces/torques/velocities, friction
    --    coefficients, object masses and inertias)
    --
    -- 2. You fake the grasping by attaching the object to the gripper via a connector. This is
    --    much easier and offers very stable results.
    --
    -- Alternative 2 is explained hereafter:
    --
    --
    -- a) In the initialization phase, retrieve some handles:
    -- 
    -- connector=sim.getObject('../attachPoint')
    -- objectSensor=sim.getObject('../attachProxSensor')
    
    -- b) Before closing the gripper, check which dynamically non-static and respondable object is
    --    in-between the fingers. Then attach the object to the gripper:
    --
    -- index=0
    -- while true do
    --     shape=sim.getObjects(index,sim.sceneobject_shape)
    --     if (shape==-1) then
    --         break
    --     end
    --     if (sim.getObjectInt32Param(shape,sim.shapeintparam_static)==0) and (sim.getObjectInt32Param(shape,sim.shapeintparam_respondable)~=0) and (sim.checkProximitySensor(objectSensor,shape)==1) then
    --         -- Ok, we found a non-static respondable shape that was detected
    --         attachedShape=shape
    --         -- Do the connection:
    --         sim.setObjectParent(attachedShape,connector,true)
    --         break
    --     end
    --     index=index+1
    -- end
    
    -- c) And just before opening the gripper again, detach the previously attached shape:
    --
    -- sim.setObjectParent(attachedShape,-1,true)
 
end 
