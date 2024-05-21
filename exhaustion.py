from utils import *
boxLength=12000
boxWidth=2566

scale_factor = 1
elevation_height = 6000
truss_height = 982
height = 1018
lower_slope_length = 2025
upper_extension_length = 0
lower_extension_length = 0
degree = 30
num=1 # 梯数
if __name__ == "__main__":
    # 初始化显示
    # display, start_display, add_menu, add_function_to_menu = init_display()

    boxNum=1
    boxes=[]
    createBox(boxNum, boxes)
    
    elevations=[]
    elevations =cs(elevation_height, height, truss_height, upper_extension_length, lower_extension_length, degree, scale_factor,num)
    all_points = calculate_split_points(elevation_height, height, upper_extension_length, degree, lower_slope_length, scale_factor)
    #每个梯子要分割的点的索引
    # split_idx = [1]
    
    split_idx = [1]  #切三段
    split_idxs=[]
    split_idxs=get_splitsets(all_points)[1:]  # 去掉第一个全空
    
    # split_idxs.append(split_idx)
    elevationSplit={}  # 记载所有扶梯的分割块 字典形式记录  0 第一个梯子

    
    
    # 切完
    for i in split_idxs:
        print(i)
        split_points=splitElevation(elevations,0,i)  
        #elevationSplit[0]  一个切完所有形状的集合  0表示选择创建的第一个梯子 目前只创建一个梯子
        # 将所有形状
        preprocess_sahpes=[]
        for bufen in elevationSplit[0]:
            center=calculate_centroid(bufen)
            origin = gp_Pnt(0, 0, 0)
            move_vector = gp_Vec(center, origin)
            # 将质心移到原点
            transformation = gp_Trsf()
            transformation.SetTranslation(move_vector)
            
            # 应用平移变换
            transformer = BRepBuilderAPI_Transform(bufen, transformation)
            moved_shape = transformer.Shape()
            preprocess_sahpes.append(moved_shape)
        placedinbox=[]
        successd=[]
        res=0
        for bufen in preprocess_sahpes:
            for x in range(-2000,12000,100):
                print(1)
                for z in range(-1000,2566,20):
                    for angle in range(0,360,3):
                        # print(1)
                        trsf = gp_Trsf()
                        # 定义一个向量，表示沿Z轴移动
                        move_vector = gp_Vec(x, 0, z)
                        # 应用平移转换
                        trsf.SetTranslation(move_vector)
                        
                        axis_dir = gp_Dir(0, 1, 0)  # 围绕 Y 轴
                        axis_pnt=calculate_centroid(bufen) # 计算质心
                        axis = gp_Ax1(axis_pnt, axis_dir)
                        trsf.SetRotation(axis, angle * 3.14159 / 180)  # 角度转弧度
                        # 使用转换工具应用转换
                        transformer = BRepBuilderAPI_Transform(bufen, trsf, True)  # True 表示复制模型进行转换
                        Shape= transformer.Shape()

                        if isInBox(Shape) :
                            flag=False
                            for placed in placedinbox:
                                # T 碰了  
                                if(check_actual_collision(Shape,placed)):
                                   flag=True 
                                   break
                            if  not flag: # 如果未碰撞 且在箱中
                                successd.append(Shape)
                                res +=1
        print("res",res) 
        with open('result.txt', 'w') as f:
            for item in successd:
                f.write("%s\n" % item)