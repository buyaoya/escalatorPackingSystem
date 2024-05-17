
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace,BRepBuilderAPI_Transform
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCC.Core.gp import gp_Pnt, gp_Vec,gp_Trsf,gp_Ax1,gp_Dir
from OCC.Display.SimpleGui import init_display
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import brepgprop
# from OCC.Display.WebGl. import init_display
import numpy as np
import math
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import topods
from OCC.Core.BRep import BRep_Tool
from OCC.Core.TopAbs import TopAbs_VERTEX


from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Splitter, BRepAlgoAPI_Fuse
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace, BRepBuilderAPI_Transform, BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeWire
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakePrism
from OCC.Core.gp import gp_Pnt, gp_Vec,gp_Trsf,gp_Ax1,gp_Dir, gp_Circ, gp_Ax2
from OCC.Core import TopAbs
from OCC.Display.WebGl.jupyter_renderer import JupyterRenderer
from OCC.Core.gp import gp_Pnt, gp_Vec, gp_Dir, gp_Pln
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeEdge
from OCC.Core.BRep import BRep_Polygon3D
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.BOPAlgo import BOPAlgo_Splitter
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRepBndLib import brepbndlib_Add
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.GC import GC_MakeArcOfCircle
from OCC.Core.Geom import Geom_Curve, Geom_Line
from OCC.Display.SimpleGui import init_display
from OCC.Core.GC import GC_MakeArcOfCircle
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve

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
    