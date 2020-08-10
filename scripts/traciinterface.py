#====================================Import Modules====================================
from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import subprocess
import random
import copy	

try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path
.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of \
        your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
from junction import Junction
from device import Device
from phaseConfig import setJunctionPhase

#===============================Generate Route File====================================

#put code to generate route file here, or better make it in some other python file, import and then run here!

#====================================Make Junctions====================================
junction_U = Junction(_id = 'U',
	dev_a_dets = ['0', '1', '2', '3', '4', '5'], 
	dev_b_dets = ['6', '7', '8', '9', '10', '11'], 
	dev_c_dets = ['54', '55', '56', '57', '58', '59'], 
	dev_d_dets = ['60', '61', '62', '63', '64', '65'],
	phaseMap = {1:1, 2:2, 3:4, 4:3})

junction_L = Junction(_id = 'L', 
	dev_a_dets = ['18', '19', '20', '21', '22', '23'], 
	dev_b_dets = ['12', '13', '14', '15', '16', '17'], 
	dev_c_dets = ['66', '67', '68', '69', '70', '71'], 
	dev_d_dets = ['24', '25', '26', '27', '28', '29'],
	phaseMap = {1:1, 2:2, 3:4, 4:3})

junction_R = Junction(_id = 'R', 
	dev_a_dets = ['30', '31', '32', '33', '34', '35'], 
	dev_b_dets = ['48', '49', '50', '51', '52', '53'], 
	dev_c_dets = ['36', '37', '38', '39', '40', '41'], 
	dev_d_dets = ['42', '43', '44', '45', '46', '47'],
	phaseMap = {1:3, 2:1, 3:2, 4:4})

#set neighbours
junction_U.neighbours = [{'junction': junction_L,'connection': ('d', 'b'), 'data':0}, {'junction': junction_R, 'connection': ('c', 'b'), 'data':0}]
junction_L.neighbours = [{'junction': junction_R,'connection': ('c', 'a'), 'data':0}, {'junction': junction_U, 'connection': ('b', 'd'), 'data':0}]
junction_R.neighbours = [{'junction': junction_L,'connection': ('a', 'c'), 'data':0}, {'junction': junction_U, 'connection': ('b', 'c'), 'data':0}]
#========================================run()=========================================
 
def run(i):

    global steps
    steps =0 
    endSimTIme = 30
    global allWaitingTime
    allWaitingTime = []
    global allTravelTime
    allTravelTime = []

    global allarrived
    allarrived =[]
    global alldeparted
    alldeparted =[] 
    fitnessParticle=[]
    phaseParticle=[]  
    
    initial=[[[50, 23, 59],0],[[23, 50, 45],1],[[52, 7, 49],1],[[22, 59, 38],0],[[25, 7, 44],0],[[32, 56, 59],0],[[15, 46, 9],1],[[10, 8, 44],0],[[6, 57, 27],0],[[40, 10, 33],1],[[45, 44, 14],0],[[59, 19, 56],0],[[22, 12, 28],1],[[8, 23, 26],0],[[45, 16, 39],1],[[56, 48, 57],0],[[46, 51, 47],0],[[17, 54, 51],1],[[54, 30, 11],0],[[36, 25,30],1],[[24, 41, 56],0],[[47, 48, 39],1],[[60, 28, 39],0],[[52, 44, 18],0],[[50, 58, 55],0],[[8, 21, 9],0],[[50, 24, 47],1],[[32, 30, 19],0],[[29, 38, 23],0],[[13,44, 45],0]]

    x=0
    print("iteration :",i)
    if i == 0 :  
	while steps < endSimTIme:

		#traci.simulationStep()
	    	#PSOphase = individual(3) 
                PSOphase=initial[x]
                x+=1
            	phaseParticle.append(PSOphase)
            	temp1 = []
      	    	temp2 = []
            	phase = (PSOphase[0][0]+PSOphase[0][1]+PSOphase[0][2])/3
	    	runDeviceDetect(phase)
            	"""
		gets data from devices for
		junctions for "time" number of simulation steps
	    	"""
            	edgeIDs = traci.edge.getIDList()
            	for j in edgeIDs:
                	temp1.append(traci.edge.getTraveltime(j))
                	temp2.append(traci.edge.getWaitingTime(j))
              
            	allWaitingTime.append(sum(temp1)/sum(alldeparted))
            	allTravelTime.append(sum(temp2)) 
                print()        
                print("----------------------------------------------------------------------")    
                print()            
		print("Step : ",steps)
            	print("Avg phase ", phase)
                print("Waiting time: ")
                print(allWaitingTime)
                print("Travel time: ")
                print(allTravelTime)
                print("Arrived cars: ", sum(allarrived))
                print("Departed cars: ", sum(alldeparted))
                print("Current Simulation time : ", traci.simulation.getTime())
	        
            	Cr = phase * (7/21)
            	if sum(allarrived) == 0:
            	    fitness= (sum(temp2)+sum(temp1)+ (sum(alldeparted)-sum(allarrived))*traci.simulation.getTime() )/ 1 + Cr
            	else:
                    fitness= (sum(temp2)+sum(temp1)+ (sum(alldeparted)-sum(allarrived))*traci.simulation.getTime() )/ (sum(allarrived))**2 + Cr
            	fitnessParticle.append(fitness)
            	print("Phases: ", PSOphase[0], "Its fitness: ",fitness)
              
	    	useAlgoAndSetPhase() 
		"""	
		use an algorithm to set the phase for the junctions
		"""
	    	prepareJunctionVectArrs()
		'''
		prepare the vehicleVectarr for junctions
		'''

	    	setJunctionPhasesInSUMO()
		'''
		set the junction's phases in the SUMO simulator
		'''

             	steps+=1
         
        value = min(fitnessParticle)
        idxg = fitnessParticle.index(value)
        pg.append(phaseParticle[idxg])
        pgf.append(value)
        allFitness.append(fitnessParticle)
        AllPSOphase.append(phaseParticle)  
	
    else: 
	steps = 0
        allWaitingTime=[]
        allTravelTime=[]
        allarrived = []
        alldeparted=[]
   
        fitnessParticle=[]
            
       #print("phases: ",AllPSOphase,"fitness: ",allFitness) 
       #print(len(AllPSOphase[-1]))
        updated = PSO(AllPSOphase[-1],allFitness[-1],pg,pgf)

        AllPSOphase.append(updated)
        h=0
          
        while steps < endSimTIme:
	     # traci.simulationStep()
              temp1 = []
      	      temp2 = []
              
              particle = updated[h] 
              #print ("len of updated",len(updated))
              h+=1
              phase = (particle[0][0]+particle[0][1]+particle[0][2])/3
              runDeviceDetect( phase)
	      """
		gets data from devices for
		junctions for "time" number of simulation steps
	      """
                
              edgeIDs = traci.edge.getIDList()

              for j in edgeIDs:
                  temp1.append(traci.edge.getTraveltime(j))
                  temp2.append(traci.edge.getWaitingTime(j))
                              
          
              if (sum(alldeparted)==0):
                 allWaitingTime.append(sum(temp1))
	      else:
		 allWaitingTime.append(sum(temp1)/sum(alldeparted))

                
              allTravelTime.append(sum(temp2)) 
              print()  
              print("-----------------------------------------------------------------")
              print()
	      print("Step : ",steps)
              print("Avg phase : ", phase)
              print("Waiting time : ")
              print(allWaitingTime)
              print("Travel time : ")
              print(allTravelTime)
              print("Arrived cars : ", sum(allarrived))
              print("Departed cars : ", sum(alldeparted))
              print("Current Simulation time : ", traci.simulation.getTime())
        
              Cr = phase * (7/21)
              if sum(allarrived) == 0:
                   fitness= (sum(temp2)+sum(temp1)+ (sum(alldeparted)-sum(allarrived))*traci.simulation.getTime() )/ 1 + Cr
              else:
                   fitness= (sum(temp2)+sum(temp1)+ (sum(alldeparted)-sum(allarrived))*traci.simulation.getTime() )/ (sum(allarrived))**2 + Cr
              fitnessParticle.append(fitness)
		
              print("Phases: ", particle[0], "Its fitness: ",fitness)
	      
              
	      useAlgoAndSetPhase() 
              """
	      use an algorithm to set the phase for the junctions
	      """
              prepareJunctionVectArrs()
	      '''
	         prepare the vehicleVectarr for junctions
	      '''

              setJunctionPhasesInSUMO()
	      '''
	       set the junction's phases in the SUMO simulator
	      '''
             
              steps+=1
        
        allFitness.append(fitnessParticle) 
        value = min(fitnessParticle)
        idx = fitnessParticle.index(value)
        pg.append(updated[idx])
        pgf.append(value)
    
    return pg,pgf 

#=========================Supplimentary functions for run()===========================
def setJunctionPhasesInSUMO():
	setJunctionPhase(junction_U, setAllRed = False)
	setJunctionPhase(junction_L, setAllRed = False)
	setJunctionPhase(junction_R, setAllRed = False)

	return

def useAlgoAndSetPhase():
	
	junction_U.update()
	junction_L.update()
	junction_R.update()
	
	return

def runDeviceDetect(phase):
	
   
	for _ in range(phase):
		
		junction_U.checkDevices()
		junction_L.checkDevices()
		junction_R.checkDevices()
                allarrived.append(traci.simulation.getArrivedNumber())
                alldeparted.append(traci.simulation.getDepartedNumber())
		traci.simulationStep()
		phase+=1
	
	return 

def prepareJunctionVectArrs():
	
	junction_U.prepareVehVectarr()
	junction_L.prepareVehVectarr()
	junction_R.prepareVehVectarr()
	
	return

def PSO(particles,fitness,pg,pgf): 

   updated=[]
   minLFit=min(fitness)
   idxL=fitness.index(minLFit)
   pB=particles[idxL]
   
   bestfit=min(pgf)
   index = pgf.index(bestfit)
   pG = pg[index]
   
   for i in range(len(particles)):
       
       updated.append(updateParticle(particles[i],pB,pG))
            
   return updated

def updateParticle(particles,pB,pG):
   
    randRou1= random.uniform(0,1)
    randRou2= random.uniform(0,1)
    w = random.uniform(0.5,1.0)
 
    vPlus1 = w*particles[1] + randRou1 * ( (pB[0][0]-particles[0][0]) + (pB[0][1]-particles[0][1]) + (pB[0][2]-particles[0][2]) )+ randRou2 * ((pG[0][0]-particles[0][0]) + (pG[0][1]-particles[0][1])  + (pG[0][2]-particles[0][2]) ) 
    
    particles[0][0]=abs(int(particles[0][0]+vPlus1))
    particles[0][1]=abs(int(particles[0][1]+vPlus1))
    particles[0][2]=abs(int(particles[0][2]+vPlus1))		
    particles[1] = vPlus1
    for i in range(3):
        while particles[0][i]>60:
           particles[0][i]=penalty(particles[0][i])
    return particles

def penalty(phase):
    phase=abs(phase-random.randint(30,60))
    return phase
  
def individual(indSize):
    indv=[]
    x=[]
    for i in range(indSize):
        x.append(random.randint(5,60))
    indv.append(x)
    rand=random.uniform(0,1)
    v=0
    if rand <0.5:
        v=0
    else:
        v=1
    indv.append(v)
    return indv


#===============================Start SUMO and call run()==============================
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    #generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    global AllPSOphase
    AllPSOphase = []
    global allFitness 
    allFitness = []
    global pg
    pg=[]
    global pgf
    pgf =[]
    BestResults = []
    for i in range (10):
        traci.start([sumoBinary, "-c", "../city.sumocfg",
                             "--tripinfo-output", "../tripinfo.xml"]) 
          
        resultsOfi = run(i)
        bestphase = resultsOfi[0]
        bestfitness= resultsOfi[1]
        print("End of iteration: ",i)
        print("Best phases :")
        print(bestphase[-1][0])
        print("----------------------------------------------------------------")
        #print("and its fitness for the entire iteration: " )
        #print(bestfitness[-1])
        BestResults.append(bestphase[-1][0])
        #print("BestResults: ",BestResults)
    print("Best phases of all iterations: ")
    print(BestResults)
    
       

    traci.close()   
