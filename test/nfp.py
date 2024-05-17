from shapely.geometry import Polygon
from shapely.affinity import translate
import matplotlib.pyplot as plt

def plot_polygons(polygons, ax, color='blue'):
    for poly in polygons:
        x, y = poly.exterior.xy
        ax.fill(x, y, alpha=0.5, fc=color, ec='black')

def generate_nfp(reference_poly, moving_poly):
    # 初始化NFP结果列表
    nfp_list = []
    
    # 获取参考多边形的所有顶点
    ref_points = list(reference_poly.exterior.coords)
    
    # 遍历移动多边形的每一个顶点
    for mx, my in moving_poly.exterior.coords:
        # 尝试将移动多边形的每个点与参考多边形的每个点对齐
        for rx, ry in ref_points:
            # 计算移动向量
            trans_x = rx - mx
            trans_y = ry - my
            
            # 平移移动多边形
            translated_poly = translate(moving_poly, xoff=trans_x, yoff=trans_y)
            
            # 检查平移后的多边形是否与参考多边形重叠
            if not reference_poly.intersects(translated_poly):
                nfp_list.append(translated_poly)

    return nfp_list

# 定义多边形
reference_poly = Polygon([[0, 6000], [2556, 6000], [8086, 3822], [7595, 3001],[2775, 4775],[0, 4775]])
moving_poly = Polygon([[8086, 3822], [12958,1035], [15157,1035], [15157,0],[12787,0],[7595,3001]])

# 生成NFP
nfp_polygons = generate_nfp(reference_poly, moving_poly)

# 绘图展示结果
fig, ax = plt.subplots()
# plot_polygons([reference_poly], ax, color='red')
plot_polygons(nfp_polygons, ax, color='black')
plt.axis('equal')
plt.show()


