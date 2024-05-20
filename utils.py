from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import brepgprop
from itertools import combinations
import numpy as np
import math
from OCC.Core.TopoDS import topods
from OCC.Core.BRep import BRep_Tool
from OCC.Core.TopAbs import TopAbs_VERTEX
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeEdge, BRepBuilderAPI_Transform, BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeWire
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakePrism
from OCC.Core.gp import gp_Pnt, gp_Vec,gp_Trsf,gp_Ax1,gp_Dir, gp_Circ, gp_Ax2, gp_Pln
from OCC.Core import TopAbs
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.BOPAlgo import BOPAlgo_Splitter
from OCC.Core.GC import GC_MakeArcOfCircle
from OCC.Core.Geom import Geom_Line
from OCC.Display.SimpleGui import init_display
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from create_prism import create_prism as cs
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common

elevations=[]
def create_prism(elevation_height, height, truss_height, upper_extension_length=0, lower_extension_length=0, angle_degrees=30, scale_factor=100,num=1):
    for n in range(num):
    
        '''
        创建一个扶梯，其形状由输入的参数定义。
        Parameters:
        elevation_height: 抬升高度
        height: 梯级高度
        truss_height: 桁架高度
        upper_extension_length: 上长度
        lower_extension_length: 下长度
        angle_degrees: 扶梯的角度
        '''
        angle = math.radians(angle_degrees)
        sin_val = math.sin(angle)
        cos_val = math.cos(angle)
        tan_val = math.tan(angle)
        upper_length = upper_extension_length + 2566
        lower_length = lower_extension_length + 2199
        p1 = gp_Pnt(0, 0, (elevation_height+height)/scale_factor)
        p2 = gp_Pnt(upper_length / scale_factor, 0, (elevation_height + height) / scale_factor)
        p3 = gp_Pnt((upper_length + elevation_height / tan_val) / scale_factor, 0, height / scale_factor)
        p4 = gp_Pnt((upper_length + elevation_height / tan_val + lower_length) / scale_factor, 0, height / scale_factor)
        p5 = gp_Pnt((upper_length + elevation_height / tan_val + lower_length) / scale_factor, 0, 0)
        p6 = gp_Pnt((upper_length + elevation_height / tan_val - height*tan_val + ((height/cos_val)-truss_height)/sin_val) / scale_factor, 0, 0)
        p7 = gp_Pnt((upper_length - truss_height * sin_val + (height - (truss_height * cos_val)) / tan_val) / scale_factor, 0, elevation_height / scale_factor)
        p8 = gp_Pnt(0, 0, elevation_height / scale_factor)
        # 要点的话 返回点就好了
        points = [p1, p2, p3, p4, p5, p6, p7, p8]
        # 创建多边形线框
        top_polygon = BRepBuilderAPI_MakePolygon()
        for pnt in points:
            top_polygon.Add(pnt)
        top_polygon.Close()

        # 创建面
        top_face = BRepBuilderAPI_MakeFace(top_polygon.Wire())
        # 沿Y轴向外推移2000单位以创建立体
        
        main_prism = BRepPrimAPI_MakePrism(top_face.Shape(),gp_Vec(0, 2000/scale_factor, 0)).Shape()
        
        # 定义圆
        circ = gp_Circ(gp_Ax2(gp_Pnt((1594.5 + upper_extension_length) / scale_factor, 0, (elevation_height + height - 2730) / scale_factor), gp_Dir(0, 1, 0)), 3000 / scale_factor)

        # 创建弧
        arc_of_circle = GC_MakeArcOfCircle(circ, 0, angle, True).Value()

        # 转换为边
        curve_edge = BRepBuilderAPI_MakeEdge(arc_of_circle).Edge()
        # 使用BRepAdaptor从边获取曲线
        adaptor1 = BRepAdaptor_Curve(curve_edge)
        first_param1, last_param1 = adaptor1.FirstParameter(), adaptor1.LastParameter()

        # 获取末端点的坐标
        start_point1 = adaptor1.Value(first_param1)
        end_point1 = adaptor1.Value(last_param1)
        normal_vector = gp_Dir(-1, 0, -1/tan_val)
        normal_line = Geom_Line(end_point1, normal_vector)
        perpendicular_edge = BRepBuilderAPI_MakeEdge(normal_line, 0, 150 / scale_factor).Edge()
        edge1 = BRepBuilderAPI_MakeEdge(Geom_Line(start_point1, gp_Dir(-1, 0, 0)), 0, 803.61 / scale_factor).Edge()
        adaptor2 = BRepAdaptor_Curve(edge1)
        last_param2 = adaptor2.LastParameter()
        end_point2 = adaptor2.Value(last_param2)
        edge2 = BRepBuilderAPI_MakeEdge(Geom_Line(end_point2, gp_Dir(0, 0, -1)), 0, 270 / scale_factor).Edge()
        adaptor3 = BRepAdaptor_Curve(edge2)
        last_param3 = adaptor3.LastParameter()
        end_point3 = adaptor3.Value(last_param3)
        adaptor4 = BRepAdaptor_Curve(perpendicular_edge)
        last_param4 = adaptor4.LastParameter()
        end_point4 = adaptor4.Value(last_param4)
        edge3 = BRepBuilderAPI_MakeEdge(end_point3, p2).Edge()
        edge4 = BRepBuilderAPI_MakeEdge(p2, end_point4).Edge()

        # 创建线框
        wire_builder = BRepBuilderAPI_MakeWire()

        # 添加所有边到线框
        wire_builder.Add(curve_edge)
        wire_builder.Add(perpendicular_edge)
        wire_builder.Add(edge1)
        wire_builder.Add(edge2)
        wire_builder.Add(edge3)
        wire_builder.Add(edge4)
        # 完成线框
        wire = wire_builder.Wire()
        # 创建面
        face = BRepBuilderAPI_MakeFace(wire, True)  # True 表示创建的面应该是平面的
        prism = BRepPrimAPI_MakePrism(face.Shape(), gp_Vec(0, 1980/scale_factor, 0)).Shape()
        
        # 定义下部圆
        lower_circ = gp_Circ(gp_Ax2(gp_Pnt((elevation_height / tan_val + 100.3 + upper_length) / scale_factor, 0, (1270 + height) / scale_factor), gp_Dir(0, 1, 0)), 1000 / scale_factor)

        # 创建弧
        lower_arc = GC_MakeArcOfCircle(lower_circ, math.pi, angle + math.pi, True).Value()

        # # 转换为边
        lower_curve_edge = BRepBuilderAPI_MakeEdge(lower_arc).Edge()
        # 使用BRepAdaptor从边获取曲线
        lower_adaptor1 = BRepAdaptor_Curve(lower_curve_edge)
        #获取末端点的参数位置
        lower_first_param1, lower_last_param1 = lower_adaptor1.FirstParameter(), lower_adaptor1.LastParameter()

        # # 获取末端点的坐标
        lower_start_point1 = lower_adaptor1.Value(lower_first_param1)
        lower_end_point1 = lower_adaptor1.Value(lower_last_param1)
        lower_edge1 = BRepBuilderAPI_MakeEdge(Geom_Line(lower_start_point1, gp_Dir(1, 0, 0)), 0, 1160.82 / scale_factor).Edge()
        lower_edge2 = BRepBuilderAPI_MakeEdge(Geom_Line(lower_end_point1, gp_Dir(-1, 0, tan_val)), 0, 304.14 / scale_factor).Edge()

        lower_adaptor2 = BRepAdaptor_Curve(lower_edge1)
        lower_last_param2 = lower_adaptor2.LastParameter()
        lower_end_point2 = lower_adaptor2.Value(lower_last_param2)
        lower_edge3 = BRepBuilderAPI_MakeEdge(Geom_Line(lower_end_point2, gp_Dir(0, 0, -1)), 0, 270 / scale_factor).Edge()

        lower_adaptor3 = BRepAdaptor_Curve(lower_edge2)
        lower_last_param3 = lower_adaptor3.LastParameter()
        lower_end_point3 = lower_adaptor3.Value(lower_last_param3)
        lower_edge4 = BRepBuilderAPI_MakeEdge(Geom_Line(lower_end_point3, gp_Dir(-1, 0, -1/tan_val)), 0, 150 / scale_factor).Edge()

        lower_adaptor4 = BRepAdaptor_Curve(lower_edge4)
        lower_last_param4 = lower_adaptor4.LastParameter()
        lower_end_point4 = lower_adaptor4.Value(lower_last_param4)
        lower_edge5 = BRepBuilderAPI_MakeEdge(lower_end_point4, p3).Edge()

        lower_adaptor5 = BRepAdaptor_Curve(lower_edge3)
        lower_last_param5 = lower_adaptor5.LastParameter()
        lower_end_point5 = lower_adaptor5.Value(lower_last_param5)
        lower_edge6 = BRepBuilderAPI_MakeEdge(lower_end_point5, p3).Edge()


        # 创建线框
        lower_wire_builder = BRepBuilderAPI_MakeWire()

        # 添加所有边到线框
        lower_wire_builder.Add(lower_curve_edge)
        lower_wire_builder.Add(lower_edge1)
        lower_wire_builder.Add(lower_edge2)
        lower_wire_builder.Add(lower_edge3)
        lower_wire_builder.Add(lower_edge4)
        lower_wire_builder.Add(lower_edge5)
        lower_wire_builder.Add(lower_edge6)

        # 完成线框
        lower_wire = lower_wire_builder.Wire()
        # 创建面
        lower_face = BRepBuilderAPI_MakeFace(lower_wire, True)  # True 表示创建的面应该是平面的
        lower_prism = BRepPrimAPI_MakePrism(lower_face.Shape(), gp_Vec(0, 1980/scale_factor, 0)).Shape()


        fuse1 = BRepAlgoAPI_Fuse(prism, main_prism).Shape()
        fused_shape = BRepAlgoAPI_Fuse(fuse1, lower_prism).Shape()

        # 创建一个转换对象
        transformation = gp_Trsf()
        # 设置转换，沿 Y 轴移动 400
        transformation.SetTranslation(gp_Vec(0, 4000*n, 0))

        # 创建一个转换操作
        transform = BRepBuilderAPI_Transform(fused_shape, transformation, True)
        # 应用转换，得到移动后的立体
        moved_shape = transform.Shape()
        elevations.append(moved_shape)
    return elevations
    
def createBox(boxNum):
    for n in range(boxNum):
        if(n==0):
            box=BRepPrimAPI_MakeBox(12000, 2000, 2566).Shape()
        else:
            start_point = gp_Pnt(0, 4000*n, 0) 
            box = BRepPrimAPI_MakeBox(start_point, 12000, 2000, 2566).Shape()
        boxs.append(box)




"""

    转换规则：
     先旋转 再下移 再移入box
     上半部 算角度旋转 下移 在左侧 往右移
     下半部 不算角度 旋转 下移  在右侧往左移

"""

# class container:
#     def __init__(self,shape,length,width,takex,):
        



def calculate_split_points(elevation_height, height, upper_extension_length=0, angle_degrees=30, lower_slope_length=2025, scale_factor=100):
    '''
    Calculate a series of split points along a slope based on given parameters.
    Returns:
        list of gp_Pnt: A list of split points.   返回的是 扶梯倾斜部分的上侧的各个切点 不包括垂直下来的下侧切点
    '''
    angle_radians = math.radians(angle_degrees)
    sin_val = math.sin(angle_radians)
    cos_val = math.cos(angle_radians)
    tan_val = math.tan(angle_radians)
    upper_length = upper_extension_length + 2566
    slope_legth = elevation_height / sin_val- lower_slope_length
    points = []
    current_distance = 0  # Start from the beginning of the slope

    # Calculate the starting point
    x_start = upper_length + elevation_height / tan_val - lower_slope_length * cos_val
    z_start = height + lower_slope_length * sin_val
    # points.append(gp_Pnt(x_start / scale_factor, 0, z_start / scale_factor))

    

    # Loop to generate points every 1200 units until reaching the lower length limit
    while current_distance  <= slope_legth - 3630:
        x = x_start - current_distance * cos_val
        z = z_start + current_distance * sin_val
        points.append(gp_Pnt(x / scale_factor, 0, z / scale_factor))
        current_distance += 1200
    return points
    
    


def get_lower_coords(split_points, elevation_height, height, truss_height, upper_extension_length=0, lower_extension_length=0, angle_degrees=30, scale_factor=1):
    angle_radians = math.radians(angle_degrees)
    sin_val = math.sin(angle_radians)
    cos_val = math.cos(angle_radians)
    tan_val = math.tan(angle_radians)
    upper_length = upper_extension_length + 2566
    lower_length = lower_extension_length + 2199
    gp_p1 = split_points[0]
    p1 = [gp_p1.X(), gp_p1.Y(), gp_p1.Z()]
    p2 = [(upper_length + elevation_height / tan_val + lower_length) / scale_factor, 0, height / scale_factor]
    p3 = [(upper_length + elevation_height / tan_val + lower_length) / scale_factor, 0, 0]
    p4 = [p1[0] - (truss_height * sin_val) / scale_factor, p1[1], p1[2] - (truss_height * cos_val) / scale_factor]
    return [p1, p2, p3, p4]

def get_upper_coords(split_points, elevation_height, height, truss_height, upper_extension_length=0, lower_extension_length=0, angle_degrees=30, scale_factor=1):
    angle_radians = math.radians(angle_degrees)
    sin_val = math.sin(angle_radians)
    cos_val = math.cos(angle_radians)
    tan_val = math.tan(angle_radians)
    upper_length = upper_extension_length + 2566
    
    gp_p1 = split_points[-1]
    p1 = [0, 0, (elevation_height+height)/scale_factor]
    p2 = [gp_p1.X(), gp_p1.Y(), gp_p1.Z()]
    p3 = [p2[0] - (truss_height * sin_val) / scale_factor, p2[1], p2[2] - (truss_height * cos_val) / scale_factor]
    p4 = [0, 0, elevation_height / scale_factor]
    p5=[(upper_length - truss_height * sin_val + (height - (truss_height * cos_val)) / tan_val) / scale_factor,0,0]
    return [p1, p2, p3, p4,p5]



#split_points 这里的是所有可切点
def get_split_idx(all_split_points, elevation_height, height, truss_height, upper_extension_length=0, lower_extension_length=0, angle_degrees=30, scale_factor=1):
    if angle_degrees == 30:
        if upper_extension_length == 0 and lower_extension_length == 0:
            if elevation_height <= 3470:
                return []
            elif elevation_height <= 6000:
                print(len(all_split_points))
                print("6000割点",[int(len(all_split_points) / 2)])
                return [int(len(all_split_points) / 2)]
            else:
                split_idx = [3] # 3 代表上段2个1200 下段3个是最极限的
                mid = len(all_split_points) - 6 #6个点表示了5段  可以当成段数
                print(len(all_split_points))
                print("mid",mid)
                while mid > 9:
                    split_idx.append(mid-3) 
                    mid -= 9 # 9代表一个箱子的最长 装入9段1200
                split_idx.append(int(len(all_split_points)-3)) # 上端是2个1200
                split_idx.sort()
                print(split_idx)
                return split_idx



# 分割第几个梯子
def splitElevation(elevations,i):
    
    # 创建分割面
    all_points = calculate_split_points(elevation_height, height, upper_extension_length, degree, lower_slope_length, scale_factor)
    split_idx=get_split_idx(all_points,elevation_height, height, truss_height, upper_extension_length, lower_extension_length, degree, scale_factor)
    if len(split_idx)!=0:
        split_points = [all_points[i] for i in split_idx] # 这里想一个切好几刀
        # split_points = [all_points[split_idx[n]]] #这改成一个切一刀，有多个梯子
        
        print("split_points------------",len(split_points))
        direction = gp_Dir(-1/math.tan(math.radians(degree)), 0, 1)
        planes = [gp_Pln(point, direction) for point in split_points]
        faces = [BRepBuilderAPI_MakeFace(plane, -10000, 10000, -10000, 10000).Shape() for plane in planes]  # 创建足够大的面

        # 初始化分割器并设置非破坏性操作
        splitter = BOPAlgo_Splitter()
        splitter.SetNonDestructive(True)
        splitter.AddArgument(elevations[i])  # 添加要被分割的立体

        # 添加所有分割工具
        for face in faces:
            splitter.AddTool(face)

        # 执行分割
        splitter.Perform()
        result = splitter.Shape()
        # 分离出所有子形状
        exp = TopExp_Explorer()
        exp.Init(result, TopAbs.TopAbs_SOLID)

        sub_shapes = []

        while exp.More():
            sub_shapes.append(exp.Current())
            exp.Next()
        elevationSplit[i]=sub_shapes
        print("elevationSplit[n]------------",elevationSplit[i])
        print("sub_shape的数量------------",len(sub_shapes))
    else: split_points=[]
    return split_points  #将切割点返回






# display.DisplayShape(sub_shapes[0],color="green",update=True,transparency=0.2)
# 0是下 1 是上----------------------------------------------------------------------------------------------！！！！！！！！！！！！

# 计算质心函数
def calculate_centroid(shape):
    properties = GProp_GProps()
    brepgprop.VolumeProperties(shape, properties)
    return properties.CentreOfMass()



# 流程  先旋转 上半部需要计算四个边界点 计算斜率 旋转到水平 或趴下  下半部 （上半部按固定角度旋转效果失败）按固定角度旋转到水平   旋转都需要计算质心  
# 然后是计算所有点的z轴最小坐标，下移这个距离  然后将上半部右移 根据所有点的x轴最左坐标  将下半部左移 根据所有点的x轴最右坐标



# 计算旋转的函数
def rotate_shape(shape, angle_degree):
    trsf = gp_Trsf()
    axis_dir = gp_Dir(0, 1, 0)  # 围绕 Y 轴
    axis_pnt=calculate_centroid(shape) # 计算质心
    axis = gp_Ax1(axis_pnt, axis_dir)
    trsf.SetRotation(axis, angle_degree * 3.14159 / 180)  # 角度转弧度
    transformer = BRepBuilderAPI_Transform(shape, trsf, True)
    return transformer.Shape()




def angle_between_lines(x1, y1, x2, y2, m2):
    # 计算斜线的斜率 m 
    m1 = (y2 - y1) / (x2 - x1)
    # 计算两条直线间夹角的正切值
    tan_theta = abs((m2 - m1) / (1 + m1 * m2))
    # 计算夹角的弧度值
    theta_radians = math.atan(tan_theta)
    # 将弧度转换为度
    theta_degrees = math.degrees(theta_radians)
    return theta_degrees



# 根据顶点将
# 函数用于遍历所有顶点的坐标  不知为什么一个点重复了6次
def vertex_coordinates(shape):
    explorer = TopExp_Explorer(shape, TopAbs_VERTEX)
    vertices = []
    while explorer.More():
        vertex = topods.Vertex(explorer.Current())
        point = BRep_Tool.Pnt(vertex)
        vertices.append((point.X(), point.Y(), point.Z()))
        explorer.Next()
    return vertices



# 函数计算下移距离  这个距离就是负的
def calculate_translation_to_ground(vertices):
    # 提取所有顶点的Z坐标
    z_coordinates = [vertex[2] for vertex in vertices]
    # 找到最小的Z坐标
    min_z = min(z_coordinates)
    # 计算需要向下移动的距离，使得最低点到达Z=0的位置
    translation_distance = -min_z
    return translation_distance




# 将在右侧外的物体移到右侧内
def moveLeftDistance(vertices):
    # 提取所有顶点的x坐标
    x_coordinates = [vertex[0] for vertex in vertices]
    # 找到最大的x坐标
    max_x = max(x_coordinates)
    # 计算需要向下移动的距离，使得最低点到达Z=0的位置
    translation_distance = max_x
    return translation_distance


# 将在左侧外的物体移到左侧内
def moveRightDistance(vertices):
    # 提取所有顶点的x坐标
    x_coordinates = [vertex[0] for vertex in vertices]
    # 找到最大的x坐标
    min_x = min(x_coordinates)
    # 计算需要向下移动的距离，使得最低点到达Z=0的位置
    translation_distance = min_x
    return translation_distance




def moveToGround(shape,translation_distance):
    trsf = gp_Trsf()
    # 定义一个向量，表示沿Z轴移动
    move_vector = gp_Vec(0, 0, translation_distance)
    # 应用平移转换
    trsf.SetTranslation(move_vector)
    # 使用转换工具应用转换
    transformer = BRepBuilderAPI_Transform(shape, trsf, True)  # True 表示复制模型进行转换
    return transformer.Shape()
    

def moveInBox(shape,groundDistance,lrDistance):
    trsf = gp_Trsf()
    # 定义一个向量，表示沿Z轴移动
    move_vector = gp_Vec(0, 0, groundDistance)
    # 应用平移转换
    trsf.SetTranslation(move_vector)
    # 使用转换工具应用转换
    transformer = BRepBuilderAPI_Transform(shape, trsf, True)  # True 表示复制模型进行转换
    groundShape= transformer.Shape()
    # 先下移再左右移动
    trsf = gp_Trsf()
    move_vector = gp_Vec(-lrDistance, 0, 0)
    trsf.SetTranslation(move_vector)
    transformer = BRepBuilderAPI_Transform(groundShape, trsf, True)
    return transformer.Shape()



from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape

    

# 判断是否在箱子内
def isInBox(shape):
    # 立体的z轴和x轴
    shapevertices = vertex_coordinates(shape)
    # 提取所有顶点的x坐标
    x_coordinates = [vertex[0] for vertex in shapevertices]
    z_coordinates = [vertex[2] for vertex in shapevertices]
    
    min_x = min(x_coordinates)
    max_x = min(x_coordinates)

    min_z = min(z_coordinates)
    max_z = min(z_coordinates)
    if(min_x>=0 and max_x<=boxLength and min_z>=0 and max_z<=boxWidth):
        return True
    return False


def check_actual_collision(up, down):
    dist_checker = BRepExtrema_DistShapeShape(up, down)
    dist_checker.Perform()
    if dist_checker.IsDone():
        min_distance = dist_checker.Value()
        if min_distance > 0:
            print("No actual collision; shapes are apart by", min_distance)
            return False
    # Proceed with BRepAlgoAPI_Common if needed
    common = BRepAlgoAPI_Common(up, down)
    common.Build()
    return common.IsDone() and not common.Shape().IsNull()



# 先处理中间切开  切开块 ，切割点(切开点拿来干嘛)
def midPack(elevationSplit):
    # 0是下 1 是上----------------------------------------------------------------------------------------------！！！！！！！！！！！！
    # display.DisplayShape(elevationSplit[0], update=True,color='black', transparency=0.2)
    fourPoint=get_upper_coords(split_points, 6000, 1018, 982, 2566, 2199, 30)
   
    for p in fourPoint:
        print(" 四个点")
        print(p)
    # 上半部 水平 3 5
    m2 =0   # 水平直线的斜率
    angleUpParallel = 360-angle_between_lines(fourPoint[4][0],fourPoint[4][2],fourPoint[2][0],fourPoint[2][2], m2)
    # print(" 感觉这点取得不对angleUpParallel:",angleUpParallel)

    rotated_down=rotate_shape(elevationSplit[0],330)
    rotated_up=rotate_shape(elevationSplit[1],330)
    # print("是否碰撞------------mid：",check_actual_collision(rotated_down,rotated_up))
    # display.DisplayShape(rotated_up, update=True,color='black', transparency=0.2)
    # display.DisplayShape(rotated_down, update=True,color='black', transparency=0.2)
    rotated_down_vertices = vertex_coordinates(rotated_down)
    rotated_up_vertices = vertex_coordinates(rotated_up)
    distanceGroundDown = calculate_translation_to_ground(rotated_down_vertices)
    distanceGroundUp = calculate_translation_to_ground(rotated_up_vertices)
    distanceRightUp = moveRightDistance(rotated_up_vertices)-0
    distanceLeftDown = moveLeftDistance(rotated_down_vertices)-12000
    inBoxdown=moveInBox(rotated_down,distanceGroundDown,distanceLeftDown)
    inBoxUp=moveInBox(rotated_up,distanceGroundUp,distanceRightUp)
    display.DisplayShape(inBoxdown, update=True,color='red', transparency=0.2)
    display.DisplayShape(inBoxUp, update=True,color='red', transparency=0.2)
    # 要不再判断坐标是否在箱子 》= 《=
    print("是否碰撞------------mid：",check_actual_collision(inBoxUp,inBoxdown))


# 中间切第一个扶梯  
# midPack(elevationSplit[1])

# 如果中间切分失败  一端最长
def bestLongPack(elevationSplit):
    # 0是下 1 是上----------------------------------------------------------------------------------------------！！！！！！！！！！！！
    # display.DisplayShape(elevationSplit[0], update=True,color='black', transparency=0.2)
    fourPoint=get_upper_coords(split_points, 6000, 1018, 982, 2566, 2199, 30)
   
    for p in fourPoint:
        print(" 四个点")
        print(p)
    # 上半部 趴下 3 4
    m2 =0   # 水平直线的斜率
    angleUpPa = 360-angle_between_lines(fourPoint[2][0],fourPoint[2][2],fourPoint[3][0],fourPoint[3][2], m2)
    print(" angleUpPa:",angleUpPa)

    rotated_down=rotate_shape(elevationSplit[0],330)
    rotated_up=rotate_shape(elevationSplit[1],angleUpPa)
    # display.DisplayShape(rotated_up, update=True,color='black', transparency=0.2)
    # 处理长段
    rotated_up_vertices = vertex_coordinates(rotated_up)
    distanceGroundUp = calculate_translation_to_ground(rotated_up_vertices)
    distanceRightUp = moveRightDistance(rotated_up_vertices)-0
    inBoxUp=moveInBox(rotated_up,distanceGroundUp,distanceRightUp)

    # 下半段是移到另一个箱子左下
    rotated_down_vertices = vertex_coordinates(rotated_down)
    distanceGroundDown = calculate_translation_to_ground(rotated_down_vertices)
    # 计算下半段minx到0的距离 借用moveRightDistance 但是减0
    distanceLeftDown = moveRightDistance(rotated_down_vertices)-0
    inBoxdown=moveInBox(rotated_down,distanceGroundDown,distanceLeftDown)
    # 将下半段移入另一个箱子  这里应该写选箱子的逻辑  如果按照箱子列表的索引 计算y轴应该移动几个4000


    trsf = gp_Trsf()
    move_vector = gp_Vec(0, 4000, 0)
    trsf.SetTranslation(move_vector)
    transformer = BRepBuilderAPI_Transform(inBoxdown, trsf, True)
    anotherBoxDown=transformer.Shape()

    display.DisplayShape(anotherBoxDown, update=True,color='red', transparency=0.2)
    display.DisplayShape(inBoxUp, update=True,color='red', transparency=0.2)
    # 要不再判断坐标是否在箱子 》= 《=
    # print("是否碰撞：",isIntersect(inBoxUp,boxs[0]))

# bestLongPack(elevationSplit[0])


# 三段切分
def threePack(elevationSplit):
    # 0是下 1 是中 2是上----------------------------------------------------------------------------------------------！！！！！！！！！！！！
    # display.DisplayShape(elevationSplit[0], update=True,color='black', transparency=0.2)
    

    rotated_down=rotate_shape(elevationSplit[0],330)
    rotated_mid=rotate_shape(elevationSplit[1],330)
    rotated_up=rotate_shape(elevationSplit[2],330)
    # display.DisplayShape(rotated_up, update=True,color='black', transparency=0.2)
    # 上段 
    rotated_up_vertices = vertex_coordinates(rotated_up)
    distanceGroundUp = calculate_translation_to_ground(rotated_up_vertices)
    distanceRightUp = moveRightDistance(rotated_up_vertices)-0
    inBoxUp=moveInBox(rotated_up,distanceGroundUp,distanceRightUp)
    #下段
    rotated_down_vertices = vertex_coordinates(rotated_down)
    distanceGroundDown = calculate_translation_to_ground(rotated_down_vertices)
    distanceLeftDown = moveLeftDistance(rotated_down_vertices)-12000
    inBoxdown=moveInBox(rotated_down,distanceGroundDown,distanceLeftDown)

    # 中半段是移到另一个箱子左下
    rotated_mid_vertices = vertex_coordinates(rotated_mid)
    distanceGroundMid = calculate_translation_to_ground(rotated_mid_vertices)
    # 计算下半段minx到0的距离 借用moveRightDistance 但是减0
    distanceLeftMid = moveRightDistance (rotated_mid_vertices)-0
    inBoxMid=moveInBox(rotated_mid,distanceGroundMid,distanceLeftMid)
    # 将下半段移入另一个箱子  这里应该写选箱子的逻辑  如果按照箱子列表的索引 计算y轴应该移动几个4000
    
    # 如果在这个中间块前有一个其他块   如果用箱子的索引的话，用个变量记录该箱子的占据x轴位置，
    # boxs[i].takex=1000


    trsf = gp_Trsf()
    move_vector = gp_Vec(1000, 4000, 0)
    trsf.SetTranslation(move_vector)
    transformer = BRepBuilderAPI_Transform(inBoxMid, trsf, True)
    anotherBoxmid=transformer.Shape()
    anotherBoxmidv=vertex_coordinates(anotherBoxmid)
    a=set()
    for vertex in anotherBoxmidv:
        a.add(vertex)
        # print(vertex)
    for i in a:
        print(i)
    display.DisplayShape(anotherBoxmid, update=True,color='red', transparency=0.2)
    display.DisplayShape(inBoxUp, update=True,color='red', transparency=0.2)
    display.DisplayShape(inBoxdown, update=True,color='red', transparency=0.2)
    # 要不再判断坐标是否在箱子 》= 《=
    print("是否碰撞：",check_actual_collision(inBoxUp,inBoxdown))
    print("是否在箱子内：",isInBox(inBoxUp))
    print("是否在箱子内：",isInBox(inBoxdown))
    print("是否在箱子内：",isInBox(anotherBoxmid))


def get_splitsets(split_points):
    # 存储所有子集的列表
    splitsets = []
    lst = list(range(len(split_points)))
    # 遍历所有可能的子集大小
    for i in range(len(lst) + 1):
        # 生成当前大小的所有子集
        for combo in combinations(lst, i):
            splitsets.append(list(combo))
    return splitsets   

boxLength=12000
boxWidth=2566

scale_factor = 1
elevation_height = 11070
truss_height = 982
height = 1018
lower_slope_length = 2025
upper_extension_length = 0
lower_extension_length = 0
degree = 30
num=2 # 梯数
if __name__ == "__main__":
    # 初始化显示
    display, start_display, add_menu, add_function_to_menu = init_display()

    boxNum=2
    boxs=[]
    createBox(boxNum)
    
    elevations=[]
    elevations =cs(elevation_height, height, truss_height, upper_extension_length, lower_extension_length, degree, scale_factor,num)
    #每个梯子要分割的点的索引
    # split_idx = [1]
    split_idx = [1,4]  #切三段
    
    elevationSplit={}  # 记载所有扶梯的分割块 字典形式记录

    split_points=splitElevation(elevations,0)  # 这个传的是实际切的点
    # display.DisplayShape(elevationSplit[0][1],color="green",update=True,transparency=0.2)  # i个梯子的上半部
    if(len(split_points)==2):
        threePack(elevationSplit[0])
    if(len(split_points)==0):
        midPack(elevationSplit[0])# 这里写直接装进去的
    # bestLongPack(elevationSplit[0])
    if(len(split_points)==1):  
        midPack(elevationSplit[0])
    for box in boxs:
        display.DisplayShape(box, color='blue', transparency=0.9)  # 箱子半透明显示
    # for elevation in elevations:
    #     display.DisplayShape(elevation, color='green', transparency=0.9)  # 箱子半透明显示

    # 显示箱子和梯形立体  


    display.View.SetScale(0.05)  #视图缩小
    # 启动显示
    start_display()