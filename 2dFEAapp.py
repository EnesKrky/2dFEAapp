from math import acos,pi,ceil
import pygame,pygame.gfxdraw
import numpy as np
import warnings

pygame.init()
warnings.filterwarnings("error")

#----FUNCTIONS----#

#Calculates new points for new triangles
def waypoint(point1,point2,pointNumber,totalPoint):
    return [round((point2[0]-point1[0])/totalPoint*pointNumber+point1[0],15),round((point2[1]-point1[1])/totalPoint*pointNumber+point1[1],15)]

#Removes repeated points in points list and modifies triangles, fixedPoints and forces lists accordingly
def removeOverdefinedPoints(Points,Triangles,FixPoints,Forces):
    setPoints=[]
    for k in Points:
        if k not in setPoints:
            setPoints.append(k)
    for k in range(len(Triangles)):
        Triangles[k]=[setPoints.index(Points[Triangles[k][0]]),setPoints.index(Points[Triangles[k][1]]),setPoints.index(Points[Triangles[k][2]])]
    for k in range(len(FixPoints)):
        FixPoints[k]=setPoints.index(Points[FixPoints[k]])
    for k in range(len(Forces)):
        Forces[k][0]=setPoints.index(Points[Forces[k][0]])  
    return setPoints,Triangles,FixPoints,Forces

#For solution, removes points which are not used by triangles
def removeDummyPoints(Points,Triangles,FixPoints,Forces):
    setPoints=[]
    for i in range(len(Points)):
        for k in Triangles:
            if (k[0]==i or k[1]==i or k[2]==i) and Points[i] not in setPoints:
                setPoints.append(Points[i])
    if len(setPoints)!=len(Points):
        for k in range(len(Triangles)):
            Triangles[k]=[setPoints.index(Points[Triangles[k][0]]),setPoints.index(Points[Triangles[k][1]]),setPoints.index(Points[Triangles[k][2]])]
        for k in range(len(FixPoints)):
            FixPoints[k]=setPoints.index(Points[FixPoints[k]])
        for k in range(len(Forces)):
            Forces[k][0]=setPoints.index(Points[Forces[k][0]])
    return setPoints,Triangles,FixPoints,Forces

#Splits selected and neighbor triangles into small triangles
def refinement(TriangleIndex,Points,Triangles,RefinementCoefficient):
    newTriangles=[]
    newPoints=[]
    pointsOnFirstLine=[waypoint(Points[Triangles[TriangleIndex][0]],Points[Triangles[TriangleIndex][1]],i,RefinementCoefficient) for i in range(1,RefinementCoefficient+1)]
    pointsOnSecondLine=[waypoint(Points[Triangles[TriangleIndex][0]],Points[Triangles[TriangleIndex][2]],i,RefinementCoefficient) for i in range(1,RefinementCoefficient+1)]
    for i in range(RefinementCoefficient):
        for k in range(i+2):
            newPoints.append(waypoint(pointsOnFirstLine[i],pointsOnSecondLine[i],k,i+1))
    lengthPoints=len(Points)
    previousRow=Triangles[TriangleIndex][0]
    for h in range(RefinementCoefficient):
        currentRow=int((h+1)*(h+2)/2)
        for j in range(h+1):
            if h==0:
                newTriangles.append([previousRow,currentRow+lengthPoints-1,currentRow+lengthPoints])
            elif h!=0 and h!=RefinementCoefficient-1 and j!=h:
                newTriangles.append([previousRow+lengthPoints-1+j,currentRow+lengthPoints-1+j,currentRow+lengthPoints+j])
                newTriangles.append([previousRow+lengthPoints-1+j,currentRow+lengthPoints+j,previousRow+lengthPoints+j])
            elif h!=0 and h!=RefinementCoefficient-1 and j==h:
                newTriangles.append([previousRow+lengthPoints-1+j,currentRow+lengthPoints-1+j,currentRow+lengthPoints+j])
            elif h==RefinementCoefficient-1:
                if j==0:
                    newTriangles.append([previousRow+lengthPoints-1+j,Triangles[TriangleIndex][1],currentRow+lengthPoints-1+j])
                    newTriangles.append([previousRow+lengthPoints-1+j,currentRow+lengthPoints-1+j,previousRow+lengthPoints+j])
                elif j==h:
                    newTriangles.append([previousRow+lengthPoints-1+j,currentRow+lengthPoints-2+j,Triangles[TriangleIndex][2]])
                else:
                    newTriangles.append([previousRow+lengthPoints-1+j,currentRow+lengthPoints-2+j,currentRow+lengthPoints-1+j])
                    newTriangles.append([previousRow+lengthPoints-1+j,currentRow+lengthPoints-1+j,previousRow+lengthPoints+j])
        previousRow=currentRow
    point1=Points[Triangles[TriangleIndex][1]]
    point2=Points[Triangles[TriangleIndex][2]]
    newPoints.remove([round(point1[0],15),round(point1[1],15)])
    newPoints.remove([round(point2[0],15),round(point2[1],15)])
    adjacent3Triangles=[None]*3
    for i in range(len(Triangles)):
        if i!=TriangleIndex and Triangles[TriangleIndex][0] in Triangles[i] and Triangles[TriangleIndex][1] in Triangles[i]:
            triList=list(Triangles[i])
            triList.remove(Triangles[TriangleIndex][0])
            triList.remove(Triangles[TriangleIndex][1])
            adjacent3Triangles[0]=[triList[0],i]
        elif i!=TriangleIndex and Triangles[TriangleIndex][0] in Triangles[i] and Triangles[TriangleIndex][2] in Triangles[i]:
            triList=list(Triangles[i])
            triList.remove(Triangles[TriangleIndex][0])
            triList.remove(Triangles[TriangleIndex][2])
            adjacent3Triangles[1]=[triList[0],i]
        elif i!=TriangleIndex and Triangles[TriangleIndex][1] in Triangles[i] and Triangles[TriangleIndex][2] in Triangles[i]:
            triList=list(Triangles[i])
            triList.remove(Triangles[TriangleIndex][1])
            triList.remove(Triangles[TriangleIndex][2])
            adjacent3Triangles[2]=[triList[0],i]
    pointsOnThirdLine=[point1]+newPoints[int(RefinementCoefficient*(RefinementCoefficient+1)/2-1):]+[point2]
    pointsOnFirstLine=[Points[Triangles[TriangleIndex][0]]]+pointsOnFirstLine
    pointsOnSecondLine=[Points[Triangles[TriangleIndex][0]]]+pointsOnSecondLine
    for i in range(len(adjacent3Triangles)):
        if adjacent3Triangles[i]!=None and i==0:
            lengthNewPoints=len(newPoints)
            newPoints+=pointsOnFirstLine
            for k in range(len(pointsOnFirstLine)-1):
                newTriangles.append([adjacent3Triangles[i][0],lengthPoints+lengthNewPoints+k,lengthPoints+lengthNewPoints+k+1])
        elif adjacent3Triangles[i]!=None and i==1:
            lengthNewPoints=len(newPoints)
            newPoints+=pointsOnSecondLine
            for k in range(len(pointsOnSecondLine)-1):
                newTriangles.append([adjacent3Triangles[i][0],lengthPoints+lengthNewPoints+k,lengthPoints+lengthNewPoints+k+1])
        elif adjacent3Triangles[i]!=None and i==2:
            lengthNewPoints=len(newPoints)
            newPoints+=pointsOnThirdLine
            for k in range(len(pointsOnThirdLine)-1):
                newTriangles.append([adjacent3Triangles[i][0],lengthPoints+lengthNewPoints+k,lengthPoints+lengthNewPoints+k+1]) 
    return newPoints,newTriangles,adjacent3Triangles

#Returns index of selected triangle
def triSelection(mpos,Points,Triangles):
    for i in Triangles:
        vec1=(Points[i[0]][0]-mpos[0],Points[i[0]][1]-mpos[1])
        vec2=(Points[i[1]][0]-mpos[0],Points[i[1]][1]-mpos[1])
        vec3=(Points[i[2]][0]-mpos[0],Points[i[2]][1]-mpos[1])
        lenVec1=(vec1[0]**2+vec1[1]**2)**0.5
        lenVec2=(vec2[0]**2+vec2[1]**2)**0.5
        lenVec3=(vec3[0]**2+vec3[1]**2)**0.5
        angle1=acos((vec1[0]*vec2[0]+vec1[1]*vec2[1])/(lenVec1*lenVec2))
        angle2=acos((vec1[0]*vec3[0]+vec1[1]*vec3[1])/(lenVec1*lenVec3))
        angle3=acos((vec3[0]*vec2[0]+vec3[1]*vec2[1])/(lenVec3*lenVec2))
        if round((angle1+angle2+angle3)*180/pi)==360:
            return Triangles.index(i)
    return None

#Returns index of selected button
def buttonSelection(Buttons,mpos,selButton):
    for i in range(len(Buttons)):
        if Buttons[i][0]<mpos[0]<Buttons[i][0]+Buttons[i][2] and Buttons[i][1]<mpos[1]<Buttons[i][1]+Buttons[i][3]:
            if selButton!=i:
                return i
            else:
                return None
    return selButton

#Adds index of selected point to cluster of selected points and creates new triangle if number of selected points is equal to 3
def pointSelectionANDtriangleCreation(Points,Triangles,mpos,selPoints,button):
    for i in range(len(Points)):
        if (Points[i][0]-mpos[0])**2+(Points[i][1]-mpos[1])**2<=25:
            if i not in selPoints:
                if button==1:
                    selPoints.append(i)
                    if len(selPoints)==3 and selPoints not in Triangles and [selPoints[0],selPoints[2],selPoints[1]] not in Triangles and [selPoints[1],selPoints[0],selPoints[2]] not in Triangles and [selPoints[1],selPoints[2],selPoints[0]] not in Triangles and [selPoints[2],selPoints[1],selPoints[0]] not in Triangles and [selPoints[2],selPoints[0],selPoints[1]] not in Triangles:
                        Triangles.append(selPoints)
                        return [],Triangles
                elif button==2:
                    if len(selPoints)==0:
                        selPoints.append(i)
                    elif len(selPoints)==1:
                        selPoints[0]=i
                elif button==3:
                    selPoints.append(i)
            else:
                selPoints.remove(i)
            break
    return selPoints,Triangles

#Converts coordinates of points to pixel coordinates
def visualpoints(Points,camPos,m,w,h):
    newPoints=[]
    for i in Points:
        newPoints.append([(i[0]-camPos[0])*m+w/2,(i[1]-camPos[1])*m+h/2])
    return newPoints

#Creates force vectors for visualization
def visualforces(Forces,Points,camPos,m,w,h):
    newForces=[]
    arrowTipLength=m/8
    for i in Forces:
        coeff=m/(i[1]**2+i[2]**2)**0.5/12
        points4=[(Points[i[0]][0]-camPos[0])*m+w/2,(Points[i[0]][1]-camPos[1])*m+h/2,(Points[i[0]][0]-camPos[0])*m+w/2-coeff*i[1]*i[3],(Points[i[0]][1]-camPos[1])*m+h/2-coeff*i[2]*i[3]]
        newForces.append(points4+[points4[0]-arrowTipLength/2*3**0.5*coeff*i[1]*i[3]/m-arrowTipLength/2*coeff*i[2]*i[3]/m,points4[1]-arrowTipLength/2*3**0.5*coeff*i[2]*i[3]/m+arrowTipLength/2*coeff*i[1]*i[3]/m,points4[0]-arrowTipLength/2*3**0.5*coeff*i[1]*i[3]/m+arrowTipLength/2*coeff*i[2]*i[3]/m,points4[1]-arrowTipLength/2*3**0.5*coeff*i[2]*i[3]/m-arrowTipLength/2*coeff*i[1]*i[3]/m])
    return newForces

#Returns new point for point cluster
def createpoint(camPos,mpos,m,w,h):
    return [round((mpos[0]-w/2)/m+camPos[0],12),round((mpos[1]-h/2)/m+camPos[1],12)]

#Creates a list of colors by going linearly from lowest value to highest value
def colors(dataList,hotColor,coldColor):
    minValue=min(dataList)
    maxValue=max(dataList)
    colorList=[]
    for i in dataList:
        colorList.append((int(hotColor[0]-(maxValue-i)/(maxValue-minValue)*(hotColor[0]-coldColor[0])),int(hotColor[1]-(maxValue-i)/(maxValue-minValue)*(hotColor[1]-coldColor[1])),int(hotColor[2]-(maxValue-i)/(maxValue-minValue)*(hotColor[2]-coldColor[2]))))
    return colorList

#Solves for displacements and stresses
def solver(Triangles,Points,PointsFixed,Forces,youngM,poissonR,thickness,hotColor,coldColor,visualScalar=1):
    usedPointsNo=len(Points)
    correctedTriangles=[] #Triangle matrix with corrected indices in terms of numbering direction of nodes
    areas=[] #Storage for areas of triangles
    for i in Triangles:
        v1=[Points[i[1]][0]-Points[i[0]][0],Points[i[1]][1]-Points[i[0]][1]]
        v2=[Points[i[2]][0]-Points[i[0]][0],Points[i[2]][1]-Points[i[0]][1]]
        crossProduct=v1[0]*v2[1]-v1[1]*v2[0]
        areas.append(abs(crossProduct)/2)
        if crossProduct>0:
            correctedTriangles.append(i)
        else:
            correctedTriangles.append([i[0],i[2],i[1]])
    D=youngM/(1-poissonR**2)*np.array([[1,poissonR,0],[poissonR,1,0],[0,0,(1-poissonR)/2]]) #Material matrix
    globalK=np.zeros((2*usedPointsNo,2*usedPointsNo)) #Global stiffness matrix for all nodes including fixed points
    Bmatrices=[] #Storage for shape matrices of elements
    Kmatrices=[] #Storage for stiffness matrices of elements
    for i,a in zip(correctedTriangles,areas):
        elementB=1/(2*a)*np.array([[Points[i[1]][1]-Points[i[2]][1],0,Points[i[2]][1]-Points[i[0]][1],0,Points[i[0]][1]-Points[i[1]][1],0],[0,Points[i[2]][0]-Points[i[1]][0],0,Points[i[0]][0]-Points[i[2]][0],0,Points[i[1]][0]-Points[i[0]][0]],[Points[i[2]][0]-Points[i[1]][0],Points[i[1]][1]-Points[i[2]][1],Points[i[0]][0]-Points[i[2]][0],Points[i[2]][1]-Points[i[0]][1],Points[i[1]][0]-Points[i[0]][0],Points[i[0]][1]-Points[i[1]][1]]])
        elementK=thickness*a*np.matmul(np.matmul(np.transpose(elementB),D),elementB)
        Bmatrices.append(elementB)
        Kmatrices.append(elementK)
    for i in range(usedPointsNo):
        for k in range(len(correctedTriangles)):
            if i in correctedTriangles[k]:
                pointIndex=correctedTriangles[k].index(i)
                #Stiffness through x direction
                globalK[2*i][correctedTriangles[k][0]*2]+=Kmatrices[k][pointIndex*2][0]
                globalK[2*i][correctedTriangles[k][0]*2+1]+=Kmatrices[k][pointIndex*2][1]
                globalK[2*i][correctedTriangles[k][1]*2]+=Kmatrices[k][pointIndex*2][2]
                globalK[2*i][correctedTriangles[k][1]*2+1]+=Kmatrices[k][pointIndex*2][3]
                globalK[2*i][correctedTriangles[k][2]*2]+=Kmatrices[k][pointIndex*2][4]
                globalK[2*i][correctedTriangles[k][2]*2+1]+=Kmatrices[k][pointIndex*2][5]
                #Stiffness through y direction
                globalK[2*i+1][correctedTriangles[k][0]*2]+=Kmatrices[k][pointIndex*2+1][0]
                globalK[2*i+1][correctedTriangles[k][0]*2+1]+=Kmatrices[k][pointIndex*2+1][1]
                globalK[2*i+1][correctedTriangles[k][1]*2]+=Kmatrices[k][pointIndex*2+1][2]
                globalK[2*i+1][correctedTriangles[k][1]*2+1]+=Kmatrices[k][pointIndex*2+1][3]
                globalK[2*i+1][correctedTriangles[k][2]*2]+=Kmatrices[k][pointIndex*2+1][4]
                globalK[2*i+1][correctedTriangles[k][2]*2+1]+=Kmatrices[k][pointIndex*2+1][5]
    globalForce=np.zeros(2*usedPointsNo) #Global force matrix
    for i in Forces:
        globalForce[2*i[0]]+=i[1]*i[3]
        globalForce[2*i[0]+1]+=i[2]*i[3]
    reversedSortedPointsFixed=list(reversed(sorted(PointsFixed)))
    filteredGlobalK=globalK #Global stiffness matrix for all nodes excluding fixed points
    filteredGlobalForce=globalForce #Global force matrix excluding fixed nodes
    pointIndices=[i for i in range(usedPointsNo)]
    filteredPointIndices=pointIndices #Index storage showing which data belongs to which node in global stiffness matrix
    for i in reversedSortedPointsFixed:
        filteredGlobalK=np.delete(filteredGlobalK,2*i+1,axis=0)
        filteredGlobalK=np.delete(filteredGlobalK,2*i,axis=0)
        filteredGlobalK=np.delete(filteredGlobalK,2*i+1,axis=1)
        filteredGlobalK=np.delete(filteredGlobalK,2*i,axis=1)
        filteredGlobalForce=np.delete(filteredGlobalForce,2*i+1)
        filteredGlobalForce=np.delete(filteredGlobalForce,2*i)
        filteredPointIndices=np.delete(filteredPointIndices,i)
    #Solution
    try:
        displacements=np.linalg.solve(filteredGlobalK,filteredGlobalForce)
    except:
        print("Something went wrong!")
        return None,None,None,None,None
    scalarDisplacements=[0]*usedPointsNo
    displacementsElement=[]
    stressesElement=[]
    for i,b in zip(correctedTriangles,Bmatrices):
        ElementDisplacements=[]
        if i[0] in filteredPointIndices:
            ElementDisplacements.append(displacements[2*list(filteredPointIndices).index(i[0])])
            ElementDisplacements.append(displacements[2*list(filteredPointIndices).index(i[0])+1])
            if scalarDisplacements[i[0]]==0:
                scalarDisplacements[i[0]]=(displacements[2*list(filteredPointIndices).index(i[0])]**2+displacements[2*list(filteredPointIndices).index(i[0])+1]**2)**0.5
        else:
            ElementDisplacements.append(0)
            ElementDisplacements.append(0)
        if i[1] in filteredPointIndices:
            ElementDisplacements.append(displacements[2*list(filteredPointIndices).index(i[1])])
            ElementDisplacements.append(displacements[2*list(filteredPointIndices).index(i[1])+1])
            if scalarDisplacements[i[1]]==0:
                scalarDisplacements[i[1]]=(displacements[2*list(filteredPointIndices).index(i[1])]**2+displacements[2*list(filteredPointIndices).index(i[1])+1]**2)**0.5
        else:
            ElementDisplacements.append(0)
            ElementDisplacements.append(0)
        if i[2] in filteredPointIndices:
            ElementDisplacements.append(displacements[2*list(filteredPointIndices).index(i[2])])
            ElementDisplacements.append(displacements[2*list(filteredPointIndices).index(i[2])+1])
            if scalarDisplacements[i[2]]==0:
                scalarDisplacements[i[2]]=(displacements[2*list(filteredPointIndices).index(i[2])]**2+displacements[2*list(filteredPointIndices).index(i[2])+1]**2)**0.5
        else:
            ElementDisplacements.append(0)
            ElementDisplacements.append(0)
        epsilon=np.matmul(b,ElementDisplacements)
        stressesElement.append(np.matmul(D,epsilon))
        displacementsElement.append(((ElementDisplacements[0]**2+ElementDisplacements[1]**2)**0.5+(ElementDisplacements[2]**2+ElementDisplacements[3]**2)**0.5+(ElementDisplacements[4]**2+ElementDisplacements[5]**2)**0.5)/3)
    vonMisesStresses=[((i[0]**2+i[1]**2+(i[0]-i[1])**2)/2+3*i[2]**2)**0.5 for i in stressesElement]
    elementStressColors=colors(vonMisesStresses,hotColor,coldColor)
    elementDisplacementColors=colors(displacementsElement,hotColor,coldColor)
    newPoints=np.array(Points)
    for i in range(len(filteredPointIndices)):
        newPoints[filteredPointIndices[i]]=[Points[filteredPointIndices[i]][0]+displacements[2*i]*visualScalar,Points[filteredPointIndices[i]][1]+displacements[2*i+1]*visualScalar]
    return list(newPoints),elementStressColors,elementDisplacementColors,vonMisesStresses,scalarDisplacements
        
#####FUNCTIONS#####

#----INITIAL PARAMETERS----#
    
width=900
height=700

screen=pygame.display.set_mode((width,height))
pygame.display.set_caption("2dFEAapp")

screen.fill((255,255,255),(0,0,width,height))
pygame.display.update()

distanceMultiplier=3000
points=[[0,0],[0,-0.075],[0.125,-0.075],[0.125,0]] #[[xPos,yPos],[...],...] in m
showPoints=True
triangles=[[0,1,2],[0,2,3]] #Indices of points which triangle consists of
forces=[[2,0,80000,1]] #[[IndexApplied,xForce,yForce,arrowDirection],[...],...] in N
cameraPos=[0.0625,-0.0375] #[xPos,yPos]

refinementIndex=None
refinementBool=False
refinementValue=2 #refinementValue^2 of new triangles are going to be created inside target triangle when refinement is applied

E=210*10**9 #Young's modulus in Pa
v=0.3 #Poisson's ratio
t=0.016 #Thickness in m

pygame.font.init()
font1=pygame.font.SysFont("amiri",20)
buttons=[[10,10,200,30,font1.render("Create Point",True,(100,100,100))],
         [10,50,200,30,font1.render("Create Triangle",True,(100,100,100))],
         [10,90,200,30,font1.render("Create Force",True,(100,100,100))],
         [10,130,200,30,font1.render("Fix Point",True,(100,100,100))],
         [10,170,200,30,font1.render("Material",True,(100,100,100))],
         [10,210,200,30,font1.render("View Solution",True,(100,100,100))]] #[[xpos,ypos,width,height,textsurface],[...],...]
selectedPoints=[]
fixedPoints=[0,1]
solutionPoints=None
solution=False
calculationChange=True
selectedButton=None

font2=pygame.font.SysFont("amiri",16)
font3=pygame.font.SysFont("amiri",int(buttons[0][2]/12))
pointInfo=None
pointStr=""
forceStr=""
materialStr=""
forceAdded=False
    
run=True
graphicalChange=True
mousePos=pygame.mouse.get_pos()
pre_dM=0

highColor=(250,120,50) #Color of highest value
lowColor=(50,120,250) #Color of lowest value
heatScaleLen=220 #Pixels
solutionViewMode=1 #1->stress,2->displacement

#####INITIAL PARAMETERS#####

#----MAIN LOOP----#

while run:
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            run=False
        elif event.type==pygame.KEYDOWN:
            if event.key==pygame.K_q:
                run=False
            elif event.key==pygame.K_r and refinementIndex!=None:
                refinementBool=True
            elif event.key==pygame.K_p:
                showPoints=not showPoints
                pointInfo=None
                graphicalChange=True
            elif event.key==pygame.K_c:
                points=[]
                triangles=[]
                fixedPoints=[]
                forces=[]
                selectedPoints=[]
                visualPoints=[]
                visualForces=[]
                refinementIndex=None
                refinementBool=False
                showPoints=True
                solutionPoints=None
                solution=False
                pointInfo=None
                visualSolutionPoints=None
                calculationChange=True
                graphicalChange=True
            elif selectedButton==0:
                key=pygame.key.name(event.key)
                prePointStr=pointStr
                if len(key)==3:
                    key=key[1:2]
                elif key=="backspace":
                    pointStr=pointStr[:-1]
                elif key=="return" and pointStr.count(",")==1:
                    try:
                        numbers=pointStr.split(",")
                        points.append([float(numbers[0]),-float(numbers[1])])
                        visualPos=visualpoints([points[-1]],cameraPos,distanceMultiplier,width,height)
                        visualPoints.append(visualPos[0])
                        pointStr=""
                    except:
                        pass
                elif len(key)==1 and font1.render(pointStr,True,(100,100,100)).get_size()[0]<buttons[0][2]*1.1:
                    try:
                        if key!="." and key!="," and key!="-":
                            number=int(key)
                            pointStr+=key
                        elif key==".":
                            pointStr+="."
                        elif key==",":
                            pointStr+=","
                        elif key=="-":
                            pointStr+="-"
                    except:
                        pass
                if prePointStr!=pointStr:
                    graphicalChange=True
            elif selectedButton==2:
                key=pygame.key.name(event.key)
                preForceStr=forceStr
                if len(key)==3:
                    key=key[1:2]
                elif key=="backspace":
                    forceStr=forceStr[:-1]
                elif key=="return" and forceStr.count(",")==1 and len(selectedPoints)>0:
                    try:
                        numbers=forceStr.split(",")
                        forces.append([selectedPoints[0],float(numbers[0]),-float(numbers[1]),1])
                        visualFor=visualforces([forces[-1]],points,cameraPos,distanceMultiplier,width,height)
                        visualForces.append(visualFor[0])
                        forceAdded=True
                        forceStr=""
                        selectedPoints=[]
                        calculationChange=True
                    except:
                        pass
                elif len(key)==1 and font1.render(forceStr,True,(100,100,100)).get_size()[0]<buttons[2][2]*1.1:
                    try:
                        if key!="." and key!="," and key!="-":
                            number=int(key)
                            forceStr+=key
                        elif key==".":
                            forceStr+="."
                        elif key==",":
                            forceStr+=","
                        elif key=="-":
                            forceStr+="-"
                    except:
                        pass
                if preForceStr!=forceStr:
                    graphicalChange=True
            elif selectedButton==4:
                key=pygame.key.name(event.key)
                preMaterialStr=materialStr
                if len(key)==3:
                    key=key[1:2]
                elif key=="backspace":
                    materialStr=materialStr[:-1]
                elif key=="return" and materialStr.count(",")==2:
                    try:
                        numbers=materialStr.split(",")
                        E=float(numbers[0])*10**9
                        v=float(numbers[1])
                        t=float(numbers[2])
                        materialStr=""
                        calculationChange=True
                    except:
                        pass
                elif len(key)==1 and font1.render(materialStr,True,(100,100,100)).get_size()[0]<buttons[4][2]*1.1:
                    try:
                        if key!="." and key!=",":
                            number=int(key)
                            materialStr+=key
                        elif key==".":
                            materialStr+="."
                        elif key==",":
                            materialStr+=","
                    except:
                        pass
                if preMaterialStr!=materialStr:
                    graphicalChange=True
            elif selectedButton==5:
                if event.key==pygame.K_1:
                    solutionViewMode=1
                    graphicalChange=True
                elif event.key==pygame.K_2:
                    solutionViewMode=2
                    graphicalChange=True
        elif event.type==pygame.MOUSEBUTTONDOWN:
            if pygame.mouse.get_pressed()[0]:
                mousePos=pygame.mouse.get_pos()
                pre_button=selectedButton
                selectedButton=buttonSelection(buttons,mousePos,selectedButton)
                if pre_button==None and selectedButton==None:
                    refinementIndex=triSelection(mousePos,visualPoints,triangles)
                elif pre_button!=selectedButton:
                    refinementIndex=None
                    pointInfo=None
                    pointStr=""
                    forceStr=""
                    selectedPoints=[]
                    if selectedButton==5:
                        solution=True
                    elif pre_button==5:
                        visualSolutionPoints=None
                        graphicalChange=True
                elif pre_button==selectedButton:
                    if selectedButton==0:
                        if buttons[0][0]<mousePos[0]<buttons[0][0]+buttons[0][2]*2+3 and buttons[0][1]<mousePos[1]<buttons[0][1]+buttons[0][3]:
                            pass
                        else:
                            points.append(createpoint(cameraPos,mousePos,distanceMultiplier,width,height))
                            visualPoints.append(mousePos)
                    elif selectedButton==1 and showPoints:
                        preTri=list(triangles)
                        selectedPoints,triangles=pointSelectionANDtriangleCreation(visualPoints,triangles,mousePos,selectedPoints,1)
                        if preTri!=triangles:
                            calculationChange=True
                    elif selectedButton==2 and forceAdded and buttons[2][0]<mousePos[0]<buttons[2][0]+buttons[2][2]*2+3 and buttons[2][1]<mousePos[1]<buttons[2][1]+buttons[2][3]:
                        forces[-1][3]*=-1
                        visualForces[-1]=visualforces([forces[-1]],points,cameraPos,distanceMultiplier,width,height)[0]
                        graphicalChange=True
                        calculationChange=True
                    elif selectedButton==2 and showPoints:
                        selectedPoints,triangles=pointSelectionANDtriangleCreation(visualPoints,triangles,mousePos,selectedPoints,2)
                    elif selectedButton==3 and showPoints:
                        preFixP=list(fixedPoints)
                        fixedPoints,triangles=pointSelectionANDtriangleCreation(visualPoints,triangles,mousePos,fixedPoints,3)
                        if preFixP!=fixedPoints:
                            calculationChange=True
                    elif selectedButton==5 and buttons[5][0]<mousePos[0]<buttons[5][0]+50 and buttons[5][1]+60<mousePos[1]<buttons[5][1]+60+heatScaleLen:
                        if solutionViewMode==1:
                            solutionViewMode=2
                        else:
                            solutionViewMode=1
                graphicalChange=True
        elif event.type==pygame.MOUSEWHEEL:
            mousePos=pygame.mouse.get_pos()
            distanceMultiplier+=event.y*pre_dM*0.1
            if distanceMultiplier<100 or distanceMultiplier>15000:
                distanceMultiplier-=event.y*pre_dM*0.1
            elif (mousePos[0]-width/2)**2+(mousePos[1]-height/2)**2>width*height/20:
                cameraPos=[cameraPos[0]+0.1/distanceMultiplier*(mousePos[0]-width/2)*abs(event.y)/event.y,cameraPos[1]+0.1/distanceMultiplier*(mousePos[1]-height/2)*abs(event.y)/event.y]
        elif event.type==pygame.MOUSEMOTION:
            if selectedButton==0 and showPoints:
                mousePos=pygame.mouse.get_pos()
                prepointInfo=pointInfo
                isMouseonPoint=False
                for i in range(len(points)):
                    if (visualPoints[i][0]-mousePos[0])**2+(visualPoints[i][1]-mousePos[1])**2<=25:
                        pointInfo=i
                        isMouseonPoint=True
                        break
                if not isMouseonPoint:
                    pointInfo=None
                if prepointInfo!=pointInfo:
                    graphicalChange=True
    if pre_dM!=distanceMultiplier:
        visualPoints=visualpoints(points,cameraPos,distanceMultiplier,width,height)
        visualForces=visualforces(forces,points,cameraPos,distanceMultiplier,width,height)
        if solutionPoints!=None and selectedButton==5:
            visualSolutionPoints=visualpoints(solutionPoints,cameraPos,distanceMultiplier,width,height)
        else:
            visualSolutionPoints=None
        pre_dM=distanceMultiplier
        graphicalChange=True
    if refinementIndex!=None and refinementValue>1 and refinementBool:
        newpoints,newtriangles,adjTris=refinement(refinementIndex,points,triangles,refinementValue)
        points+=newpoints
        count=0
        triVal=[]
        for i in adjTris:
            if i!=None:
                triVal.append(triangles[i[1]])
                if i[1]<refinementIndex:
                    count+=1
        for i in triVal:
            triangles.remove(i)
        triangles=triangles[:refinementIndex-count]+newtriangles+triangles[refinementIndex-count+1:]
        points,triangles,fixedPoints,forces=removeOverdefinedPoints(points,triangles,fixedPoints,forces)
        visualPoints=visualpoints(points,cameraPos,distanceMultiplier,width,height)
        refinementBool=False
        refinementIndex=None
        graphicalChange=True
        calculationChange=True
    if graphicalChange:
        screen.fill((240,240,240),(0,0,width,height))
        if selectedButton!=5 or solutionPoints==None:
            #Draws selected triangle
            if refinementIndex!=None:
                pygame.draw.polygon(screen,(50,200,50),(visualPoints[triangles[refinementIndex][0]],visualPoints[triangles[refinementIndex][1]],visualPoints[triangles[refinementIndex][2]]))
            #Draws triangles
            for i in triangles:
                pygame.draw.lines(screen,(0,0,0),True,(visualPoints[i[0]],visualPoints[i[1]],visualPoints[i[2]]))
            #Draws points
            if showPoints:
                for k in visualPoints:
                    if visualPoints.index(k) in selectedPoints:
                        pygame.gfxdraw.filled_circle(screen,int(k[0]),int(k[1]),3,(50,200,50))
                        pygame.gfxdraw.aacircle(screen,int(k[0]),int(k[1]),3,(50,200,50))
                    elif visualPoints.index(k) in fixedPoints:
                        pygame.gfxdraw.filled_circle(screen,int(k[0]),int(k[1]),3,(210,180,20))
                        pygame.gfxdraw.aacircle(screen,int(k[0]),int(k[1]),3,(210,180,20))
                    else:
                        pygame.gfxdraw.filled_circle(screen,int(k[0]),int(k[1]),3,(200,50,50))
                        pygame.gfxdraw.aacircle(screen,int(k[0]),int(k[1]),3,(200,50,50))
        #Draws points and triangles calculated for solution
        if distanceMultiplier>=100:
            lineWidth=3
        else:
            lineWidth=round(0.03*distanceMultiplier)
        if visualSolutionPoints!=None:
            if solutionViewMode==1:
                triColors=triStressColors
            else:
                triColors=triDisplacementColors
            for i,c in zip(solTriangles,triColors):
                pygame.draw.polygon(screen,c,(visualSolutionPoints[i[0]],visualSolutionPoints[i[1]],visualSolutionPoints[i[2]]))
                pygame.draw.lines(screen,(0,0,0),True,(visualSolutionPoints[i[0]],visualSolutionPoints[i[1]],visualSolutionPoints[i[2]]),lineWidth)
        #Draws arrows representing force vectors
        for i in visualForces:
            pygame.draw.line(screen,(0,50,200),i[:2],i[2:4],lineWidth)
            pygame.draw.polygon(screen,(0,50,200),(i[:2],i[4:6],i[6:]))
            pygame.draw.aalines(screen,(0,50,200),True,(i[:2],i[4:6],i[6:]))
        #Draws information box of the point where mouse cursor is over
        if pointInfo!=None:
            info="p"+str(pointInfo)+"("+str(round(points[pointInfo][0],3))+","+str(-round(points[pointInfo][1],3))+")"
            infoRender=font2.render(info,True,(100,100,100))
            infoRenderDimensions=infoRender.get_size()
            pygame.draw.rect(screen,(130,200,190),(ceil(visualPoints[pointInfo][0]),ceil(visualPoints[pointInfo][1]-infoRenderDimensions[1]*1.3),ceil(infoRenderDimensions[0]*1.3),ceil(infoRenderDimensions[1]*1.3)))
            screen.blit(infoRender,(visualPoints[pointInfo][0]+infoRenderDimensions[0]*0.15,visualPoints[pointInfo][1]-infoRenderDimensions[1]*1.15))
            pygame.draw.lines(screen,(0,0,0),True,((ceil(visualPoints[pointInfo][0]),ceil(visualPoints[pointInfo][1])),(ceil(visualPoints[pointInfo][0]),ceil(visualPoints[pointInfo][1]-infoRenderDimensions[1]*1.3)),(ceil(visualPoints[pointInfo][0]+infoRenderDimensions[0]*1.3),ceil(visualPoints[pointInfo][1]-infoRenderDimensions[1]*1.3)),(ceil(visualPoints[pointInfo][0]+infoRenderDimensions[0]*1.3),ceil(visualPoints[pointInfo][1]))))
        #Draws buttons
        for h in buttons:
            fontDimensions=h[4].get_size()
            if selectedButton!=buttons.index(h):
                pygame.draw.rect(screen,(190,200,170),h[:4])
                screen.blit(h[4],(h[0]+h[2]/2-fontDimensions[0]/2,h[1]+h[3]/2-fontDimensions[1]/2))
            else:
                if buttons.index(h)==selectedButton==5 and solution and calculationChange:
                    fontRender=font1.render("Being Solved...",True,(100,100,100))
                    fontDimensions=fontRender.get_size()
                    pygame.draw.rect(screen,(50,200,50),h[:4])
                    screen.blit(fontRender,(h[0]+h[2]/2-fontDimensions[0]/2,h[1]+h[3]/2-fontDimensions[1]/2))
                else:
                    pygame.draw.rect(screen,(50,200,50),h[:4])
                    screen.blit(h[4],(h[0]+h[2]/2-fontDimensions[0]/2,h[1]+h[3]/2-fontDimensions[1]/2))
                    if buttons.index(h)==selectedButton==5 and solutionPoints!=None:
                        for i in range(buttons[5][1]+60,buttons[5][1]+60+heatScaleLen):
                            pygame.draw.rect(screen,(int(highColor[0]-(heatScaleLen-i+buttons[5][1]+60)/heatScaleLen*(highColor[0]-lowColor[0])),int(highColor[1]-(heatScaleLen-i+buttons[5][1]+60)/heatScaleLen*(highColor[1]-lowColor[1])),int(highColor[2]-(heatScaleLen-i+buttons[5][1]+60)/heatScaleLen*(highColor[2]-lowColor[2]))),(buttons[5][0],i,50,1))
                        pygame.draw.lines(screen,(0,0,0),True,((buttons[5][0],buttons[5][1]+60),(buttons[5][0]+50,buttons[5][1]+60),(buttons[5][0]+50,buttons[5][1]+60+heatScaleLen),(buttons[5][0],buttons[5][1]+60+heatScaleLen)))
                        pygame.draw.line(screen,(0,0,0),(buttons[5][0]+50,buttons[5][1]+60),(buttons[5][0]+65,buttons[5][1]+60))
                        pygame.draw.line(screen,(0,0,0),(buttons[5][0]+50,buttons[5][1]+60+heatScaleLen/4),(buttons[5][0]+65,buttons[5][1]+60+heatScaleLen/4))
                        pygame.draw.line(screen,(0,0,0),(buttons[5][0]+50,buttons[5][1]+60+heatScaleLen/2),(buttons[5][0]+65,buttons[5][1]+60+heatScaleLen/2))
                        pygame.draw.line(screen,(0,0,0),(buttons[5][0]+50,buttons[5][1]+60+heatScaleLen*3/4),(buttons[5][0]+65,buttons[5][1]+60+heatScaleLen*3/4))
                        pygame.draw.line(screen,(0,0,0),(buttons[5][0]+50,buttons[5][1]+60+heatScaleLen),(buttons[5][0]+65,buttons[5][1]+60+heatScaleLen))
                        if solutionViewMode==1:
                            minValue=min(totalStresses)/10**6
                            maxValue=max(totalStresses)/10**6
                            unit=" MPa"
                        else:
                            minValue=min(totalDisplacements)*1000
                            maxValue=max(totalDisplacements)*1000
                            unit=" mm"
                        #Minimum value
                        heatScaleMinRender=font2.render("%g"%minValue,True,(50,50,50))
                        heatScaleMinDimensions=heatScaleMinRender.get_size()
                        screen.blit(heatScaleMinRender,(buttons[5][0]+70,buttons[5][1]+60-heatScaleMinDimensions[1]/2+2))
                        #Value at point1
                        heatScalePoint1Render=font2.render("%g"%(minValue+(maxValue-minValue)/4),True,(50,50,50))
                        heatScalePoint1Dimensions=heatScalePoint1Render.get_size()
                        screen.blit(heatScalePoint1Render,(buttons[5][0]+70,buttons[5][1]+60+heatScaleLen/4-heatScalePoint1Dimensions[1]/2+2))
                        #Value at midpoint
                        heatScaleMidRender=font2.render("%g"%(minValue+(maxValue-minValue)/2)+unit,True,(50,50,50))
                        heatScaleMidDimensions=heatScaleMidRender.get_size()
                        screen.blit(heatScaleMidRender,(buttons[5][0]+70,buttons[5][1]+60+heatScaleLen/2-heatScaleMidDimensions[1]/2+2))
                        #Value at point2
                        heatScalePoint2Render=font2.render("%g"%(minValue+(maxValue-minValue)*3/4),True,(50,50,50))
                        heatScalePoint2Dimensions=heatScalePoint2Render.get_size()
                        screen.blit(heatScalePoint2Render,(buttons[5][0]+70,buttons[5][1]+60+heatScaleLen*3/4-heatScalePoint2Dimensions[1]/2+2))
                        #Maximum value
                        heatScaleMaxRender=font2.render("%g"%maxValue,True,(50,50,50))
                        heatScaleMaxDimensions=heatScaleMaxRender.get_size()
                        screen.blit(heatScaleMaxRender,(buttons[5][0]+70,buttons[5][1]+60+heatScaleLen-heatScaleMaxDimensions[1]/2+2))
            pygame.draw.lines(screen,(0,0,0),True,((h[0],h[1]),(h[0]+h[2],h[1]),(h[0]+h[2],h[1]+h[3]),(h[0],h[1]+h[3])))
            if buttons.index(h)==selectedButton==0:
                pygame.draw.rect(screen,(190,200,170),(h[0]+h[2]+3,h[1],h[2],h[3]))
                pygame.draw.lines(screen,(0,0,0),True,((h[0]+h[2]+3,h[1]),(h[0]+h[2]+3+h[2],h[1]),(h[0]+h[2]+3+h[2],h[1]+h[3]),(h[0]+h[2]+3,h[1]+h[3])))
                if pointStr!="":
                    pointXYRender=font2.render(pointStr,True,(0,50,200))
                else:
                    pointXYRender=font3.render("Point in X,Y format [m]",True,(160,160,160))
                pointXYRenderDimensions=pointXYRender.get_size()
                screen.blit(pointXYRender,(ceil(h[0]+h[2]+5),ceil(h[1]+h[3]/2-pointXYRenderDimensions[1]/2)))
            elif buttons.index(h)==selectedButton==2:
                pygame.draw.rect(screen,(190,200,170),(h[0]+h[2]+3,h[1],h[2],h[3]))
                pygame.draw.lines(screen,(0,0,0),True,((h[0]+h[2]+3,h[1]),(h[0]+h[2]+3+h[2],h[1]),(h[0]+h[2]+3+h[2],h[1]+h[3]),(h[0]+h[2]+3,h[1]+h[3])))
                if forceStr!="":
                    forceXYRender=font2.render(forceStr,True,(0,50,200))
                else:
                    forceXYRender=font3.render("Force in X,Y format [N]",True,(160,160,160))
                forceXYRenderDimensions=forceXYRender.get_size()
                screen.blit(forceXYRender,(ceil(h[0]+h[2]+5),ceil(h[1]+h[3]/2-forceXYRenderDimensions[1]/2)))
            elif buttons.index(h)==selectedButton==4:
                pygame.draw.rect(screen,(190,200,170),(h[0]+h[2]+3,h[1],h[2],h[3]))
                pygame.draw.lines(screen,(0,0,0),True,((h[0]+h[2]+3,h[1]),(h[0]+h[2]+3+h[2],h[1]),(h[0]+h[2]+3+h[2],h[1]+h[3]),(h[0]+h[2]+3,h[1]+h[3])))
                if materialStr!="":
                    materialRender=font2.render(materialStr,True,(0,50,200))
                else:
                    materialRender=font3.render("E [GPa],v,t [m]",True,(160,160,160))
                materialRenderDimensions=materialRender.get_size()
                screen.blit(materialRender,(ceil(h[0]+h[2]+5),ceil(h[1]+h[3]/2-materialRenderDimensions[1]/2)))
        pygame.display.update()
        graphicalChange=False
    if solution:
        presolutionPoints=solutionPoints
        if calculationChange:
            if len(triangles)>1 and len(points)>0 and len(fixedPoints)>1 and len(forces)>0:
                triSave=list(triangles)
                fixSave=list(fixedPoints)
                forceSave=list(forces)
                solPoints,solTriangles,solFixPoints,solForces=removeDummyPoints(points,triangles,fixedPoints,forces)
                triangles=triSave
                fixedPoints=fixSave
                forces=forceSave
                solutionPoints,triStressColors,triDisplacementColors,totalStresses,totalDisplacements=solver(solTriangles,solPoints,solFixPoints,solForces,E,v,t,highColor,lowColor,20)
            calculationChange=False
        if (not np.array_equal(presolutionPoints,solutionPoints) or visualSolutionPoints==None) and solutionPoints!=None:
            visualSolutionPoints=visualpoints(solutionPoints,cameraPos,distanceMultiplier,width,height)
        graphicalChange=True
        solution=False

#####MAIN LOOP#####
        
pygame.quit()
