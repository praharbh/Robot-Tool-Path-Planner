"""
	Author:    Prahar Bhatt
	Created:   10.25.2019

	Center for Advanced Manufacturing, University of Southern California.
"""

from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Display.SimpleGui import *
from OCC.Core.StlAPI import StlAPI_Writer
from OCC.Core.TopoDS import topods_Face
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.IFSelect import IFSelect_RetDone, IFSelect_ItemsByEntity
from OCC.Core.GeomAbs import GeomAbs_Plane, GeomAbs_Cylinder
from OCC.Extend.TopologyUtils import TopologyExplorer
from OCC.Core.ShapeAnalysis import ShapeAnalysis_Surface, shapeanalysis_GetFaceUVBounds
from OCC.Core.gp import gp_Pnt2d, gp_Pnt, gp_Vec, gp_Ax1, gp_Dir
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
import random
import math
import copy

def recognize_clicked(shap, *kwargs):
	for shp in shap:
		if shp not in surf:
			surf.append(shp)
			display.DisplayShape(shp, color='WHITE', transparency= 0.5, update=True)

		else:
			surf.remove(shp)
			display.DisplayShape(shp, color='BLACK', update=True)

def get_point(bsurf, u, v):
	pnt = gp_Pnt()
	bx = gp_Vec()
	by = gp_Vec()
	# pnt = bsurf.Value(u, v)
	bsurf.D1(u, v, pnt, bx, by)
	bz = bx.Crossed(by)
	bx=bx.Normalized()
	by=by.Normalized()
	bz=bz.Normalized()
	return pnt,bx,by,bz

def generate_TCP():
	global tool_tcp_pnts, tool_tcp_vxs, tool_tcp_vys , tool_tcp_vzs
	tool_tcp_pnts=[]
	tool_tcp_vxs=[]
	tool_tcp_vys=[]
	tool_tcp_vzs=[]
	numperface=5
	num1=round(abs(math.sqrt(numperface)))
	num2=round(abs(math.sqrt(numperface)))
	numofrots=8
	for sur in surf:
		umin, umax, vmin, vmax = shapeanalysis_GetFaceUVBounds(sur)
		bsur = BRepAdaptor_Surface(sur, True)
		for i in range(0,num1+1):
			for j in range(0,num2+1):
				u = umin+i*(umax-umin)/num1
				v = vmin+j*(vmax-vmin)/num2
				point,Vx,Vy,Vz = get_point(bsur, u, v)

				flag=0
				for apoint in tool_tcp_pnts:
					if abs(point.Distance(apoint))< 0.1:
						flag=1
						break

				if flag==0:
					for k in range(0,numofrots):
						ang=2*math.pi/numofrots
						tool_tcp_pnts.append(point)
						Vx=Vx.Rotated(gp_Ax1(point,gp_Dir(Vz)),ang)
						tool_tcp_vxs.append(Vx)
						# print(Vx.X(),',',Vx.Y(),',',Vx.Z())
						Vy=Vy.Rotated(gp_Ax1(point,gp_Dir(Vz)),ang)
						tool_tcp_vys.append(Vy)
						tool_tcp_vzs.append(Vz)	

	display_coord(tool_tcp_pnts,tool_tcp_vxs,tool_tcp_vys,tool_tcp_vzs)
	display.DisplayMessage(gp_Pnt(100,100,100),'TCP #:'+str(len(tool_tcp_pnts)),height=None, message_color=(1,1,1), update=True)

def read_step(fileloc):
    step_reader = STEPControl_Reader()
    step_reader.ReadFile(fileloc)
    step_reader.TransferRoot()
    shape = step_reader.Shape()
    return shape

def shape_selection(type):
	if type=='F':
		display.SetSelectionModeFace()
	elif type=='V':
		display.SetSelectionModeVertex()
	elif type=='E':
		display.SetSelectionModeEdge()
	else:
		display.SetSelectionModeShape()


def display_coord(pnts,vecx,vecy,vecz):
    for i in range(0,len(pnts)):
        rayx = BRepBuilderAPI_MakeEdge(pnts[i], gp_Pnt((gp_Vec(pnts[i].XYZ()) + vecx[i]*10).XYZ())).Edge()
        rayy = BRepBuilderAPI_MakeEdge(pnts[i], gp_Pnt((gp_Vec(pnts[i].XYZ()) + vecy[i]*10).XYZ())).Edge()
        rayz = BRepBuilderAPI_MakeEdge(pnts[i], gp_Pnt((gp_Vec(pnts[i].XYZ()) + vecz[i]*10).XYZ())).Edge()
        display.DisplayShape(rayx, color = 'RED', update = False)
        display.DisplayShape(rayy, color = 'GREEN', update = False)
        display.DisplayShape(rayz, color = 'BLUE1', update = False)

def custom_menu():
	add_menu('Actions')
	add_function_to_menu('Actions', generate_TCP)
	add_function_to_menu('Actions', save_CSV)

def save_CSV():
    if not len(tool_tcp_pnts) == 0:
        text_file = open(('CSV/'+tool_name+'.csv'), "w")
        for i in range(0,len(tool_tcp_pnts)):
            text_file.write(str(round(tool_tcp_pnts[i].X(),3)))
            text_file.write(",")
            text_file.write(str(round(tool_tcp_pnts[i].Y(),3)))
            text_file.write(",")
            text_file.write(str(round(tool_tcp_pnts[i].Z(),3)))

            text_file.write(",")
            text_file.write(str(round(tool_tcp_vxs[i].X(),3)))
            text_file.write(",")
            text_file.write(str(round(tool_tcp_vxs[i].Y(),3)))
            text_file.write(",")
            text_file.write(str(round(tool_tcp_vxs[i].Z(),3)))

            text_file.write(",")
            text_file.write(str(round(tool_tcp_vys[i].X(),3)))
            text_file.write(",")
            text_file.write(str(round(tool_tcp_vys[i].Y(),3)))
            text_file.write(",")
            text_file.write(str(round(tool_tcp_vys[i].Z(),3)))

            text_file.write(",")
            text_file.write(str(round(tool_tcp_vzs[i].X(),3)))
            text_file.write(",")
            text_file.write(str(round(tool_tcp_vzs[i].Y(),3)))
            text_file.write(",")
            text_file.write(str(round(tool_tcp_vzs[i].Z(),3)))

            text_file.write("\r\n")
        text_file.close()
        print (len(tool_tcp_pnts), "points have been saved")
        

if __name__ == "__main__":
	display, start_display, add_menu, add_function_to_menu = init_display()

	global surf, tool_name
	surf = []

	tool_name='l3_asm'
	tool_shape = read_step(('CAD/'+tool_name+'.stp'))
	display.DisplayShape(tool_shape, color='BLACK', update=True)

	shape_selection('F')
	display.register_select_callback(recognize_clicked)

	custom_menu()

	start_display()
