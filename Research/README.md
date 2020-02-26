
# AMCR Control Script
This is a formatted description of how the AMCR control script works, non-threaded scripts and threaded scripts as well as how to modify the AMCR control script to achieve a desired effect.
## Non-threaded script information
Non-threaded scripts are a collection of blocking functions that have a specific execution order, there are many additional functions that this script doesn't use, such as sysCall_sensing and callback-related functions.
### How the robot makes use of this scripting type
The ACMR robot relies on sysCall_init to initialize variables and setup tables, and sysCall_actuation which is called every simulation tick (50ms by default) that is used for the main bulk of the code. Ideally the sensing portion should be split up into sysCall_sensing however because the variables used for joint positions need to be modifed in this, the developers have grouped them together into the actuation function.
## Main head link script
 The main heads script is split up into the two functions identified above.
 ### sysCall_init
 The init portion is mainly used for variable declarations, setting up the horizontal and vertical movement tables for the respondable links and obtaining handles of the various components the robot needs to either control or access.

I've added some additional code to see the front camera, and to display a console for debugging the outputs
```lua
frontCam=sim.getObjectHandle('ACMR_camera')
frontView=sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
sim.adjustView(frontView,frontCam,0)
```
```lua
myconsole=sim.auxiliaryConsoleOpen('Text_Debugger',10,0,{0.5,0.9},{0.2,0.2},NULL,NULL)
sim.auxiliaryConsoleShow(myconsole,1)
sim.auxiliaryConsolePrint(myconsole,'--INIT--')
```
 ### sysCall_actuation
 The actuation portion is split up into 5 stages, the first which takes the simulation time as a variable in calculating the vpos for motion (hpos is 0 until we detect that we're able to move in a 3d space), later on the script clater on the script calls sim.setJointTargetPosition on the v/h joints to actually move the robot.
 ```lua
 --calculating the randomised angle for the link to go to
 s=s+sim.getSimulationTimeStep()*(2*math.cos(sim.getSimulationTime()*0.5)+0.3)
 vPos=maxHAngle*(math.pi/180)*math.sin(s*2.5)
 hPos=0
...
sim.setJointTargetPosition(vJoint,vPos)
sim.setJointTargetPosition(hJoint,hPos)
 ```
The second section calculates friction on the robot as well as the gravity then applies that using addForceAndTorque (this can be duplicated and modified to also calculate hydrodynamics of the robot).
p[3] is also the z axis of the robot.
```lua
--gravity
p=sim.getObjectPosition(bodyS,-1)
cm=(0.05-p[3])/0.05
if (cm>1.05) then cm=1.05 end
if (cm<0) then cm=0 end
sim.addForceAndTorque(body,{0,0,9.81*cm})
--friction
linV,angV=sim.getVelocity(body)
m=sim.getObjectMatrix(bodyS,-1)
m[4]=0
m[8]=0
m[12]=0
mi=simGetInvertedMatrix(m)
linV=sim.multiplyVector(mi,linV)
linV[1]=0
linV=sim.multiplyVector(m,linV)
f={linV[1]*mass*str*cm,linV[2]*mass*str*cm,linV[3]*mass*str*cm}
sim.addForceAndTorque(body,f)
```
The third section is used for checking if the links Z axis is less than 0.02 and set the phase to 2 (meaning the robot is now dealing with both vertical and horizontal motion instead of vertical. Later on this is used for randomising the hPos variable based on a phaseTransition variable that will increase every simulation tick the robot is able to go "downwards", capping at 1.
```lua
--check if we're able to go down
if (p[3]<-0.02) then
    phase=2
end
--slowly change the hPos variable based on the amount of time we've been able to go down for
if (phase==2) then
    if (phaseTransition<1) then
        phaseTransition=phaseTransition+0.02
    end
    hPos=phaseTransition*30*(math.pi/180)*math.sin(s*1.4)
 end
```
The fourth section is used for sensing, this section can be split up into 2 main portions, the first checks if the proximity sensor detects anything, which will set the proxCounter to 27 and set the v/hdir variables to an angle that would effectively turn the robot away from the obstacle, inverting the angle if v/hpos are negative respectfully. 
```lua
if (proxCounter==0)and(sim.readProximitySensor(prox)==1) then
    proxCounter=proxTiming*4
    vdir=math.pi*45/180
    if (vPos<0) then vdir=-vdir end
    hdir=math.pi*30/180
    if (hPos<0) then hdir=-hdir end
end
```
The second portion in this section is used to tweak the v/hpos based on the amount of simulation ticks has passed since detecting an obstacle, this is to prevent the snake from bumping into itself or the object. Each section modifes the vPos, then checks if we're dealing with horizontal motion as well and at the end clamps those variables so it doesn't turn too sharply.
```lua
if (proxCounter~=0) then
    if (proxCounter>3*proxTiming) then
        t=(proxCounter-3*proxTiming)/proxTiming
        vPos=vPos*t+vdir*(1-t)
        if (phase==2) then hPos=hPos*t+hdir*(1-t) end
    else
        if (proxCounter>proxTiming) then
            vPos=vdir
            if (phase==2) then hPos=hdir end
        else
            t=proxCounter/proxTiming
            vPos=vPos*(1-t)+vdir*t
            if (phase==2) then hPos=hPos*(1-t)+hdir*t end
        end
    end
    if (math.abs(vPos)>maxHAngle) then vPos=maxHAngle*vPos/math.abs(vPos) end
    proxCounter=proxCounter-1
end
```
The last section clears the existing movement in the movement table and inserts the current v/h pos for each responsable link to follow, this is done by sim.sendData on packed float/int tables to transfer the data between the head and the respondable links.
```lua
table.remove(verticalMovementTable,tableLength)
table.remove(horizontalMovementTable,tableLength)
...
table.insert(verticalMovementTable,1,vPos)
table.insert(horizontalMovementTable,1,hPos)

sim.sendData(sim.handle_tree,0,'ACMR_vmt',sim.packFloatTable(verticalMovementTable))
sim.sendData(sim.handle_tree,0,'ACMR_hmt',sim.packFloatTable(horizontalMovementTable))
sim.sendData(sim.handle_tree,0,'ACMR_imd',sim.packInt32Table({interModuleDelay}))
sim.sendData(sim.handle_tree,0,'ACMR_str',sim.packFloatTable({str}))
```
 ### sysCall_cleanup
 The original cleanup script didn't initially contain any clean up code, however I had to add some code to remove the camera and console at the end of the simulation.
## Respondable script
Each respondable links goal is to follow the head links motion with a delay, with some additional friction and gravity calculations similarly how the main head does.
### sysCall_init
The init functions of the respondables are a lot more bare-bones than the main link, setting up the relevant object handles with the only main variation being the modulePos which is used to find out which index of the motion table to access.
```lua
modulePos=1
```
### sysCall_actuation
The actuation function of the respondables can be split up into 3 sections, the first section takes all of the packed data from the head link and stores it into variables to be used for actuating and calculating friction/gravitational forces.
```lua
data=sim.receiveData(0,'ACMR_vmt')
verticalMovementTable=sim.unpackFloatTable(data)
data=sim.receiveData(0,'ACMR_hmt')
horizontalMovementTable=sim.unpackFloatTable(data)
data=sim.receiveData(0,'ACMR_imd')
interModuleDelay=sim.unpackInt32Table(data)[1]
data=sim.receiveData(0,'ACMR_str')
```
The second section takes the str from the main link (not entirely sure why since it's static -20) and calculates the same gravitation and friction forces as the main link.
```lua
str=sim.unpackFloatTable(data)[1]
p=sim.getObjectPosition(bodyS,-1)
cm=(0.05-p[3])/0.05
if (cm>1.05) then cm=1.05 end
if (cm<0) then cm=0 end
sim.addForceAndTorque(body,{0,0,9.81*cm})

linV,angV=sim.getVelocity(body)
m=sim.getObjectMatrix(bodyS,-1)
m[4]=0
m[8]=0
m[12]=0
mi=simGetInvertedMatrix(m)
linV=sim.multiplyVector(mi,linV)
linV[1]=0
linV=sim.multiplyVector(m,linV)
f={linV[1]*mass*str*cm,linV[2]*mass*str*cm,linV[3]*mass*str*cm}
sim.addForceAndTorque(body,f)
```
The final section takes the accesses the specific modules index and delay in the table to get it's horizontal and vertical motion, since the calculations are already done in the main link this section is really simple.
```lua
sim.setJointTargetPosition(vJoint,verticalMovementTable[modulePos*interModuleDelay])
sim.setJointTargetPosition(hJoint,horizontalMovementTable[modulePos*interModuleDelay])
```