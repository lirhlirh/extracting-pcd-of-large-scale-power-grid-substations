import open3d as o3d
import Basic_Info

def Vis_Crop_PCD(filePathInput):
    print("Demo for manual geometry cropping")
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left click for polygon selection")
    print("4) Press 'C' to get a selected geometry and to save it")
    print("5) Press 'F' to switch to freeview mode")
    # pcd = o3d.io.read_point_cloud(filePathInput+"/output_000_Device.pcd")
    pcd = o3d.io.read_point_cloud(filePathInput+'/preExtract/Cropped_Grd_SOR/35/35_Device.pcd')
    # pcd = o3d.io.read_point_cloud(filePathInput+'/preExtract/Cropped_Grd_SOR/220/220_Device.pcd')
    # pcd = o3d.io.read_point_cloud(filePathInput+'/preExtract/Cropped_Grd_SOR/35/35_Device.pcd')
    # pcd = o3d.io.read_point_cloud(filePathInput+'/preExtract/Cropped_Grd_SOR/500/500_Device.pcd')
    o3d.visualization.draw_geometries_with_editing([pcd])

if __name__=='__main__':
    pathBase, regions = Basic_Info.Basic_Info()
    Vis_Crop_PCD(pathBase)
