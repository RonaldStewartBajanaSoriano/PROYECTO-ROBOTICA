# Type help("robolink") or help("robodk") for more information
# Press F5 to run the script
# Note: you do not need to keep a copy of this file, your python script is saved with the station
from robolink import *    # API to communicate with RoboDK
from robodk import *      # basic matrix operations
import time
import copy
# Use RoboDK API as RL
RDK = Robolink()
rob = robodk
# define default approach distance
APPROACH = 100

# gather robot, tool and reference frames from the station
robot               = RDK.Item('UR10e1', ITEM_TYPE_ROBOT)
robot2              = RDK.Item('UR3e', ITEM_TYPE_ROBOT)
tool                = RDK.Item('GripperA', ITEM_TYPE_TOOL)
frame_pallet        = RDK.Item('PalletA', ITEM_TYPE_FRAME)
frame_conv          = RDK.Item('ConveyorReference', ITEM_TYPE_FRAME)
frame_conv_moving   = RDK.Item('MovingRef', ITEM_TYPE_FRAME)
frame_pedestal   = RDK.Item('pedestal', ITEM_TYPE_FRAME)


# tomar las partes creadas
part1               = RDK.Item('Parte 1', ITEM_TYPE_OBJECT)
part2               = RDK.Item('Parte 2', ITEM_TYPE_OBJECT)
part3               = RDK.Item('Parte 3', ITEM_TYPE_OBJECT)
part4               = RDK.Item('Parte 4', ITEM_TYPE_OBJECT)
part5               = RDK.Item('Parte 5', ITEM_TYPE_OBJECT)
part6               = RDK.Item('Parte 6', ITEM_TYPE_OBJECT)
part7               = RDK.Item('Parte 7', ITEM_TYPE_OBJECT)
part8               = RDK.Item('Parte 8', ITEM_TYPE_OBJECT)
listaparte=[part1,part2,part3,part4,part5,part6,part7,part8]



# gather targets
target_pallet_safe = RDK.Item('PalletApproachA', ITEM_TYPE_TARGET)
target_conv_safe = RDK.Item('ConvApproachA', ITEM_TYPE_TARGET)
target_conv = RDK.Item('Put Conveyor', ITEM_TYPE_TARGET)
target_mesa= RDK.Item('Mesa', ITEM_TYPE_TARGET)

#targets ur3e
inicio= RDK.Item('inicio', ITEM_TYPE_TARGET)
pos1= RDK.Item('pos1', ITEM_TYPE_TARGET)


# get variable parameters
SIZE_BOX = RDK.getParam('SizeBox')
SIZE_PALLET = RDK.getParam('SizePallet')
SIZE_BOX_XYZ = [float(x.replace(' ','')) for x in SIZE_BOX.split(',')]
SIZE_PALLET_XYZ = [float(x.replace(' ','')) for x in SIZE_PALLET.split(',')]
SIZE_BOX_Z = SIZE_BOX_XYZ[2] # the height of the boxes is important to take into account when approaching the positions

def box_calc(size_xyz, pallet_xyz):
    """Calculates a list of points to store parts in a pallet"""
    [size_x, size_y, size_z] = size_xyz
    [pallet_x, pallet_y, pallet_z] = pallet_xyz    
    xyz_list = []
    for h in range(int(pallet_z)):
        for j in range(int(pallet_y)):
            for i in range(int(pallet_x)):
                xyz_list = xyz_list + [[(i+0.5)*size_x, (j+0.5)*size_y, (h+0.5)*size_z]]
    return xyz_list

def TCP_On(toolitem):
    """Attaches the closest object to the toolitem Htool pose,
    It will also output appropriate function calls on the generated robot program (call to TCP_On)"""
    toolitem.AttachClosest()
    toolitem.RDK().RunMessage('Set air valve on')
    toolitem.RDK().RunProgram('TCP_On()');
        
def TCP_Off(toolitem, itemleave=0):
    """Detaches the closest object attached to the toolitem Htool pose,
    It will also output appropriate function calls on the generated robot program (call to TCP_Off)"""
    toolitem.DetachAll(itemleave)
    toolitem.RDK().RunMessage('Set air valve off')
    toolitem.RDK().RunProgram('TCP_Off()');

def elipse(): 
    RefTar = pos1.Pose() 
    b1 = rob.pose_2_xyzrpw(RefTar)     
    t = 0     
    x0= b1[0]     
    y0= b1[1]     
    z0= b1[2]     
    a=70          
    robot2.MoveJ(RefTar)   
    while (t <=2*pi):         
        pos = transl(x0 + a*cos(t), y0 + a*sin(t), z0)*roty(-pi)         
        robot2.MoveJ(pos)            
        t +=pi/360 


#RL.Set_Simulation_Speed(500);
#RL.Set_Simulation_Mode(1)

# calculate an array of positions to get/store the parts
parts_positions = box_calc(SIZE_BOX_XYZ, SIZE_PALLET_XYZ)

# Calculate a new TCP that takes into account the size of the part (targets are set to the center of the box)
tool_xyzrpw = tool.PoseTool()*transl(0,0,SIZE_BOX_Z/2)
tool_tcp = robot.AddTool(tool_xyzrpw, 'TCP A')

# ---------------------------------------------------------------------------------
# -------------------------- PROGRAM START ----------------------------------------

robot.setPoseTool(tool_tcp)
nparts = len(parts_positions)
i = nparts-1
partref2 = RDK.Item('box100mm2')
partref2.Copy()
while i >= 0:
    # ----------- take a part from the pallet ------
    # get the xyz position of part i
    robot.setPoseFrame(frame_pallet)
    part_position_i = parts_positions[i]
    target_i = transl(part_position_i)*rotx(pi)
    target_i_app = target_i * transl(0,0,-(APPROACH+SIZE_BOX_Z))    
    # approach to the pallet
    robot.MoveJ(target_pallet_safe)
    # get the box i
    robot.MoveJ(target_i_app)
    robot.MoveL(target_i)
    TCP_On(tool) # attach the closest part
    robot.MoveL(target_i_app)
    robot.MoveJ(target_pallet_safe)

    # ----------- place the box i on the convegor ------
    # Robot A deja piezas en espacio de trabajo del robot C
    robot.setPoseFrame(frame_pedestal)
    target_mesa_pose = target_mesa.Pose()*transl(0,0,-SIZE_BOX_Z/2)
    target_mesa_app = target_mesa_pose*transl(0,0,-APPROACH)
    robot.MoveJ(target_conv_safe)
    robot.MoveJ(target_mesa_app)
    robot.MoveL(target_mesa_pose)
    TCP_Off(tool, frame_pedestal) # detach an place the object in the moving reference frame of the conveyor
    robot.MoveL(target_mesa_app)
    robot.MoveJ(target_conv_safe)
    time.sleep(0.5)

    # Robot C realiza el fresado
    robot2.MoveJ(inicio)
    elipse()
    robot2.MoveJ(inicio)
    
    # Al finalizar el fresado se eliminan las piezas cuadradas
    parte='Part '+str(i+1)
    objects = RDK.ItemList(ITEM_TYPE_OBJECT, False)
    for item in objects:
        if item.Name().startswith(parte):
            item.Delete()

    # Reemplazo de las piezas cuadradas por las cilindricas
    teletransporta=listaparte[-i+nparts-1]
    teletransporta.setPose(transl([0,0,305]))
    
    # Robot A mueve las piezas trabajadas a la banda trasportadora
    robot.MoveL(target_mesa_pose)
    TCP_On(tool)
    robot.setPoseFrame(frame_conv)
    target_conv_pose = target_conv.Pose()*transl(0,0,-SIZE_BOX_Z/2)
    target_conv_app = target_conv_pose*transl(0,0,-APPROACH)
    robot.MoveJ(target_conv_app)
    robot.MoveL(target_conv_pose)
    TCP_Off(tool, frame_conv_moving) # detach an place the object in the moving reference frame of the conveyor
    robot.MoveL(target_conv_app)
    robot.MoveJ(target_conv_safe)
    
    int(i)
    i = i - 1