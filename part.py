"""
    Author:    Prahar Bhatt
    Created:   10.25.2019

    Center for Advanced Manufacturing, University of Southern California.
"""

from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Display.SimpleGui import *
from OCC.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.gp import gp_Pnt, gp_Vec, gp_Trsf, gp_Dir, gp_Ax1, gp_Mat, gp_Quaternion, gp_GTrsf, gp_XYZ
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
import time
import pandas as pd
import copy
import time
import networkx as nx
import random
import math
import numpy as np
# import matplotlib.pyplot as plt

class assert_isdone(object):
    def __init__(self, to_check, error_statement):
        self.to_check = to_check
        self.error_statement = error_statement
    def __enter__(self, ):
        if self.to_check.IsDone():
            pass
        else:
            raise AssertionError(self.error_statement)
    def __exit__(self, assertion_type, value, traceback):
        pass

def collision(shp1, shp2):
    bdss = BRepExtrema_DistShapeShape(shp1, shp2)
    bdss.Perform()
    with assert_isdone(bdss, 'failed computing minimum distances'):
        min_dist = bdss.Value()
        # min_dist_shp1, min_dist_shp2 = [], []
        # for i in range(1, bdss.NbSolution()+1):
        #     min_dist_shp1.append(bdss.PointOnShape1(i))
        #     min_dist_shp2.append(bdss.PointOnShape2(i))
    # if min_dist>0:
    #     flag=0
    # else:
    #     flag=1
    return min_dist

def read_step(fileloc):
    step_reader = STEPControl_Reader()
    step_reader.ReadFile(fileloc)
    step_reader.TransferRoot()
    shape = step_reader.Shape()
    return shape

def htransform(shape,pnt,vx,vy,vz):
    vx = vy.Crossed(vz)
    vy = vz.Crossed(vx)
    vx.Normalize()
    vy.Normalize()
    vz.Normalize()
    tshape=copy.deepcopy(shape)
    rot = gp_Mat(vx.X(),vy.X(),vz.X(),vx.Y(),vy.Y(),vz.Y(),vx.Z(),vy.Z(),vz.Z())
    quat = gp_Quaternion(rot)
    disp = gp_Vec(gp_Pnt(0, 0, 0),pnt)
    trsf = gp_Trsf()
    trsf.SetTransformation(quat,disp)
    aniloc = TopLoc_Location(trsf)
    loc = aniloc*tshape.Location()
    tshape.Location(loc)
    return aniloc, tshape

def htransform2(tpnt,tvx,tvy,tvz,pnt,vx,vy,vz):
    tvx = tvy.Crossed(tvz)
    tvy = tvz.Crossed(tvx)
    tvx.Normalize()
    tvy.Normalize()
    tvz.Normalize()
    tpnt=gp_Pnt((gp_Vec(tpnt.XYZ()) + tvz*0.1).XYZ())
    # tpnt=gp_Pnt((gp_Vec(tpnt.XYZ()) + tvz*2).XYZ())

    vx = vy.Crossed(vz)
    vy = vz.Crossed(vx)
    vx.Normalize()
    vy.Normalize()
    vz.Normalize()
    
    # tshape=copy.deepcopy(shape)
    
    Tt=gp_GTrsf(gp_Mat(tvx.X(),tvy.X(),tvz.X(),tvx.Y(),tvy.Y(),tvz.Y(),tvx.Z(),tvy.Z(),tvz.Z()),gp_XYZ(tpnt.X(),tpnt.Y(),tpnt.Z()))
    Tt.Invert()
    rott = gp_Mat(tvx.X(),tvy.X(),tvz.X(),tvx.Y(),tvy.Y(),tvz.Y(),tvx.Z(),tvy.Z(),tvz.Z())
    rott.Transpose()
    quatt = gp_Quaternion(rott)
    dispt = gp_Vec(Tt.TranslationPart().X(),Tt.TranslationPart().Y(),Tt.TranslationPart().Z())
    trsft = gp_Trsf()
    trsft.SetTransformation(quatt,dispt)
    # loct = TopLoc_Location(trsft)
    # loct = loct*tshape.Location()
    # tshape.Location(loct)

    rotp = gp_Mat(vx.X(),vy.X(),vz.X(),vx.Y(),vy.Y(),vz.Y(),vx.Z(),vy.Z(),vz.Z())
    quatp = gp_Quaternion(rotp)
    dispp = gp_Vec(gp_Pnt(0,0,0),pnt)
    trsfp = gp_Trsf()
    trsfp.SetTransformation(quatp,dispp)
    trsfo=trsfp.Multiplied(trsft)
    # locp = TopLoc_Location(trsfp)
    # locp = locp*tshape.Location()
    # tshape.Location(locp)
    loco = TopLoc_Location(trsfo)
    # loco = loco*tshape.Location()
    # tshape.Location(loco)
    
    return loco

# def animation(tpnts,txdir,tydir,tzdir,shape,pnts,xdir,ydir,zdir,extshape,step):
#     ais_shape = display.DisplayShape(shape, color='BLACK', transparency= 0, update=True)
#     for i in range(0,len(pnts),step):
#         j=1
#         toploc, XXX = htransform2(tpnts[j],txdir[j],tydir[j],tzdir[j],shape,pnts[i],xdir[i],ydir[i],zdir[i])
#         display.Context.SetLocation(ais_shape, toploc)
#         display.Context.UpdateCurrentViewer()
#         t=time.time()
#         print('collision',collision(extshape,XXX))
#         print('time',(time.time()-t))

def animate_tool():
    for i in range(0,len(path_tool_loc)):
        display.Context.SetLocation(ais_shape, path_tool_loc[i])
        display.Context.UpdateCurrentViewer()
        time.sleep(0.05)

def display_coords(pnts,vecx,vecy,vecz):
    for i in range(0,len(pnts)):
        rayx = BRepBuilderAPI_MakeEdge(pnts[i], gp_Pnt((gp_Vec(pnts[i].XYZ()) + vecx[i]*10).XYZ())).Edge()
        rayy = BRepBuilderAPI_MakeEdge(pnts[i], gp_Pnt((gp_Vec(pnts[i].XYZ()) + vecy[i]*10).XYZ())).Edge()
        rayz = BRepBuilderAPI_MakeEdge(pnts[i], gp_Pnt((gp_Vec(pnts[i].XYZ()) + vecz[i]*10).XYZ())).Edge()
        display.DisplayShape(rayx, color = 'RED', update = False)
        display.DisplayShape(rayy, color = 'GREEN', update = False)
        display.DisplayShape(rayz, color = 'BLUE1', update = False)

def display_coord(pnt,vecx,vecy,vecz):
    rayx = BRepBuilderAPI_MakeEdge(pnt, gp_Pnt((gp_Vec(pnt.XYZ()) + vecx*10).XYZ())).Edge()
    rayy = BRepBuilderAPI_MakeEdge(pnt, gp_Pnt((gp_Vec(pnt.XYZ()) + vecy*10).XYZ())).Edge()
    rayz = BRepBuilderAPI_MakeEdge(pnt, gp_Pnt((gp_Vec(pnt.XYZ()) + vecz*10).XYZ())).Edge()
    display.DisplayShape(rayx, color = 'RED', update = False)
    display.DisplayShape(rayy, color = 'GREEN', update = False)
    display.DisplayShape(rayz, color = 'BLUE1', update = False)

def read_pnts(pnts_loc):
    readCSV = pd.read_csv(pnts_loc,sep=',',header=None,dtype=float)
    points = readCSV.values
    pnts=[]
    xdir=[]
    ydir=[]
    zdir=[]
    for j in range(0,len(points)):
        pnts.append(gp_Pnt(points[j][0],points[j][1],points[j][2]))
        xdir.append(gp_Vec(points[j][3],points[j][4],points[j][5]))
        ydir.append(gp_Vec(points[j][6],points[j][7],points[j][8]))
        zdir.append(gp_Vec(points[j][9],points[j][10],points[j][11]))
    return pnts, xdir, ydir, zdir

def save_CSV():
    if not len(path_tool_loc) == 0:
        text_file = open(('CSV/'+part_name+'.csv'), "w")
        for i in range(0,len(path_tool_loc)):
            dummy_tranf = path_tool_loc[i].Transformation()
            dummy_disp = dummy_tranf.TranslationPart()
            dummy_rot = dummy_tranf.VectorialPart()
            dummy_vx = dummy_rot.Column(1)
            dummy_vx.Normalize()
            dummy_vy = dummy_rot.Column(2)
            dummy_vy.Normalize()
            dummy_vz = dummy_rot.Column(3)
            dummy_vz.Normalize()
            text_file.write(str(round(dummy_disp.X(),3)))
            text_file.write(",")
            text_file.write(str(round(dummy_disp.Y(),3)))
            text_file.write(",")
            text_file.write(str(round(dummy_disp.Z(),3)))

            text_file.write(",")
            text_file.write(str(round(dummy_vx.X(),3)))
            text_file.write(",")
            text_file.write(str(round(dummy_vx.Y(),3)))
            text_file.write(",")
            text_file.write(str(round(dummy_vx.Z(),3)))

            text_file.write(",")
            text_file.write(str(round(dummy_vy.X(),3)))
            text_file.write(",")
            text_file.write(str(round(dummy_vy.Y(),3)))
            text_file.write(",")
            text_file.write(str(round(dummy_vy.Z(),3)))

            text_file.write(",")
            text_file.write(str(round(dummy_vz.X(),3)))
            text_file.write(",")
            text_file.write(str(round(dummy_vz.Y(),3)))
            text_file.write(",")
            text_file.write(str(round(dummy_vz.Z(),3)))
            text_file.write("\r\n")
        text_file.close()
        print (len(tool_tcp_pnts), "points have been saved")

def custom_menu():
    add_menu('Actions')
    add_function_to_menu('Actions', animate_tool)
    add_function_to_menu('Actions', save_CSV)

if __name__ == "__main__":
    global tool_shape, path_tool_loc, ais_shape, part_name, tool_name
    path_tool_loc=[]
    part_name='wp6'
    tool_name='l2_asm'

    display, start_display, add_menu, add_function_to_menu = init_display()

    part_shape = read_step('CAD/'+part_name+'.stp')
    display.DisplayShape(part_shape, color=None, transparency= 0, update=True)
    part_pnts, part_xdir, part_ydir, part_zdir = read_pnts('CSV/'+part_name+'.csv')
    display_coords(part_pnts,part_xdir, part_ydir, part_zdir)

    tool_shape = read_step('CAD/'+tool_name+'.stp')
    tool_tcp_pnts,tool_tcp_vxs,tool_tcp_vys,tool_tcp_vzs=read_pnts('CSV/'+tool_name+'.csv')
    # display.DisplayShape(tool_shape, color=None, transparency= 0, update=True)
    # display_coords(tool_tcp_pnts,tool_tcp_vxs,tool_tcp_vys,tool_tcp_vzs)
    ais_shape = display.DisplayShape(tool_shape, color='BLACK', transparency= 0, update=True)
    custom_menu()

    ################################################################################
    if False:
        print('Genrating Sequence Path...')
        
        for i in range(0,50):#len(part_pnts),1):
            for j in range(0,len(tool_tcp_pnts)):
            # while (True):
                # j=random.randint(0,len(tool_tcp_pnts)-1)
                ntool_loc = htransform2(tool_tcp_pnts[j],tool_tcp_vxs[j],tool_tcp_vys[j],tool_tcp_vzs[j],part_pnts[i], part_xdir[i], part_ydir[i], part_zdir[i])
                # print('query')
                c=collision(part_shape, tool_shape.Moved(ntool_loc))
                # print('done')
                if c>0.095:
                    print(c,',',j,',',i)
                    path_tool_loc.append(ntool_loc)
                    break
        print('Path Genrated')

    ###############################################################################
    if True:
        tic1=time.time()
        nopnt=len(part_pnts)
        print('# of Waypoints:',nopnt)
        notcp=len(tool_tcp_pnts)
        print('# of Tcps:',notcp)
        nonodes=nopnt*notcp
        print('Genrating Graph...')
        DG=nx.DiGraph()
        node_list=list(range(1,nonodes))
        DG.add_nodes_from(node_list)

        for i in range(1,nonodes-notcp+1):
            ll=math.floor(i/notcp)
            kk=(i%notcp)-1
            if kk==-1:
                kk=notcp-1
            otool_loc = htransform2(tool_tcp_pnts[kk],tool_tcp_vxs[kk],tool_tcp_vys[kk],tool_tcp_vzs[kk],part_pnts[ll], part_xdir[ll], part_ydir[ll], part_zdir[ll])
            otool_tranf = otool_loc.Transformation()
            otool_rot = otool_tranf.VectorialPart()
            otool_disp = otool_tranf.TranslationPart()
            otool_vx = otool_rot.Column(1)
            otool_vx.Normalize()
            otool_vy = otool_rot.Column(2)
            otool_vy.Normalize()
            otool_vz = otool_rot.Column(3)
            otool_vz.Normalize()
            j=(math.ceil(i/notcp)*notcp)+1
            l=math.ceil(i/notcp)
            for k in range(0,notcp):
                DG.add_edge(i,j+k)
                ntool_loc = htransform2(tool_tcp_pnts[k],tool_tcp_vxs[k],tool_tcp_vys[k],tool_tcp_vzs[k],part_pnts[l], part_xdir[l], part_ydir[l], part_zdir[l])
                ntool_tranf = ntool_loc.Transformation()
                ntool_rot = ntool_tranf.VectorialPart()
                ntool_disp = ntool_tranf.TranslationPart()
                ntool_vx = ntool_rot.Column(1)
                ntool_vx.Normalize()
                ntool_vy = ntool_rot.Column(2)
                ntool_vy.Normalize()
                ntool_vz = ntool_rot.Column(3)
                ntool_vz.Normalize()
                world_ang = abs(math.acos(ntool_vz.Dot(gp_XYZ(0,0,-1).Normalized())))/(2*math.pi)
                world_dist = abs(gp_XYZ(0,ntool_vz.Y(),ntool_vz.Z()).Modulus())
                rel_ang = abs(3-ntool_vx.Dot(otool_vx)-ntool_vy.Dot(otool_vy)-ntool_vz.Dot(otool_vz))/(3*2*math.pi)
                rel_dist = abs((ntool_disp-otool_disp).Modulus())/1000
                # weigh = rel_dist + rel_ang
                weigh = world_ang #+ world_dist
                DG[i][j+k]['weight']=weigh
        print('Graph Genertated')
        toc1=time.time()-tic1
        print('Graph Generation Time:',toc1)

        tic2=time.time()
        print('Collison Check and Search...')

        for s in range(0,notcp):
            stool_loc = htransform2(tool_tcp_pnts[s],tool_tcp_vxs[s],tool_tcp_vys[s],tool_tcp_vzs[s],part_pnts[0], part_xdir[0], part_ydir[0], part_zdir[0])
            c=collision(part_shape, tool_shape.Moved(stool_loc))
            if c>0.095:
                break
        s=s+1
        print('Start:',s)

        for t in range(0,notcp):
            ttool_loc = htransform2(tool_tcp_pnts[t],tool_tcp_vxs[t],tool_tcp_vys[t],tool_tcp_vzs[t],part_pnts[nopnt-1], part_xdir[nopnt-1], part_ydir[nopnt-1], part_zdir[nopnt-1])
            c=collision(part_shape, tool_shape.Moved(ttool_loc))
            if c>0.095:
                break
        t=(t+1)+nonodes-notcp
        print('Target:',t)

        pnt_list=list(range(0,nopnt))
        path_c=0
        vis_node=[]
        rem_node=[]
        deltcps=[0] * (nopnt+1)
        count=0
        while(path_c<0.095):
            try:
                path=nx.dijkstra_path(DG,s,t) 
                path_length=nx.dijkstra_path_length(DG,s,t)
            except:
                break
            count=count+1
            # if count>notcp:
            #     break
            print('Iteration #:',count)
            path_c=1

            for l in range(1,nopnt-1):
                if path[l] in vis_node:
                    continue
                else:
                    vis_node.append(path[l])

                k=(path[l]%notcp)-1
                if k==-1:
                    k=notcp-1

                ctool_loc=htransform2(tool_tcp_pnts[k],tool_tcp_vxs[k],tool_tcp_vys[k],tool_tcp_vzs[k],part_pnts[l], part_xdir[l], part_ydir[l], part_zdir[l])
                c=collision(part_shape, tool_shape.Moved(ctool_loc))

                if c<0.095:
                    path_c=0
                    node_list.remove(path[l])
                    for m in range(0,notcp):
                        DG[(l-1)*notcp+(m+1)][path[l]]['weight']=DG[(l-1)*notcp+(m+1)][path[l]]['weight']+100000000
                    # DG[path[l-1]][path[l]]['weight']=1000000000000000000
                    # DG.remove_node(path[l])
                    
                    # if path[l] in rem_node:
                    #     print('Error!!!')
                    # rem_node.append(path[l])

                    # deltcps[l]=deltcps[l]+1
                    # if deltcps[l]>=notcp:
                    #     # pnt_list.remove(l)
                    #     n=1
                    #     while(deltcps[l-n]>=notcp):
                    #         n=n+1
                    #     m=1
                    #     # # while(deltcps[l+m]>=notcp):
                    #     # #     m=m+1
                    #     DG.add_edge(path[l-n],path[l+m])
                    #     DG[path[l-n]][path[l+m]]['weight']=0
                    # print(path[l],',',c,',',k,',',l)

        print('Path Ready to Animate')
        toc2=time.time()-tic2
        print('Collision and Search:',toc2)
        toc=toc1+toc2
        print('Total Time Taken:',toc)


        for l in range(0,nopnt):
            if path[l] not in node_list:
                continue
            k=(path[l]%notcp)-1
            if k==-1:
                k=notcp-1
            ftool_loc=htransform2(tool_tcp_pnts[k],tool_tcp_vxs[k],tool_tcp_vys[k],tool_tcp_vzs[k],part_pnts[l], part_xdir[l], part_ydir[l], part_zdir[l])
            path_tool_loc.append(ftool_loc)
        print('Successful Waypoints:', len(path_tool_loc))
    # nx.draw_circular(DG)
    # plt.show()

    ################################################################################
    # display.DisplayShape(tool_shape, color=None, transparency= 0, update=True)
    # display_coord(gp_Pnt(0,0,0),gp_Vec(1,0,0),gp_Vec(0,1,0),gp_Vec(0,0,1))
    # display_coord(gp_Pnt(200,200,200),gp_Vec(0,1,0),gp_Vec(0,0,1),gp_Vec(1,0,0))
    # j=0
    # display_coord(tool_tcp_pnts[j],tool_tcp_vxs[j],tool_tcp_vys[j],tool_tcp_vzs[j])
    # xxx,ntool_shape = htransform2(tool_tcp_pnts[j],tool_tcp_vxs[j],tool_tcp_vys[j],tool_tcp_vzs[j],tool_shape,gp_Pnt(200,200,200),gp_Vec(0,1,0),gp_Vec(0,0,1),gp_Vec(1,0,0))
    # display.DisplayShape(ntool_shape, color='BLACK', transparency= 0.5, update=True)

    ##############################################################################################
    # tool_tshape = htransform(tool_shape,tempt,tempx,tempy,tempz)
    # value = minimum_distance(part_shape,tool_tshape)
    # display_coord(tempt,tempx,tempy,tempz)
    # display.DisplayShape(part_shape, color='BLACK', transparency= 0.5, update=False)
    
    # display.Erase()

    # step_reader.ReadFile('CAD/wp1.stp')
    # step_reader.TransferRoot()
    # shape2 = step_reader.Shape()

    # display.SetSelectionModeFace()
    # display.register_select_callback(recognize_clicked)
    # display.DisplayShape(shape1, color='BLUE', transparency= 0.2, update=True)

    # t=time.time()
    # value = minimum_distance(shape1,shape2)
    # t=time.time()-t
    # print(value)
    # print(t)
    # add_menu('sup?')
    # add_function_to_menu('sup?', minimum_distance(shape1,shape2))

    start_display()