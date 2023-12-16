from search.algorithms import State
from search.map import Map
import getopt
import sys
import heapq as heap 
import math 

def A_star(start,goal,gridded_map):
    OPENLIST=[]
    CLOSED = {}
    CLOSED[start.state_hash()] = start
    heap.heappush(OPENLIST,start)
    expand_nodes =0
    while len(OPENLIST): 
        n=heap.heappop(OPENLIST)

        if n == goal:
            return n.get_g(),expand_nodes
        for child in gridded_map.successors(n):
            hash_value = child.state_hash()
            deltaX = abs(goal.get_x()-child.get_x())
            deltaY = abs(goal.get_y()-child.get_y())
            g_d=1.5*min(deltaX,deltaY)+abs(deltaX-deltaY)
            ab=child.get_g()+g_d
            if hash_value not in CLOSED:
                child.set_cost(ab)
                heap.heappush(OPENLIST,child)
                CLOSED[hash_value] = child
                expand_nodes = expand_nodes +1
            elif hash_value in CLOSED and ab< CLOSED[hash_value].get_cost():
                child.set_cost(ab)
                heap.heappush(OPENLIST,child)
                CLOSED[hash_value] = child
        
    return -1,expand_nodes

def BiAstar(start, goal, gridded_map):
        openf=[]
        openb=[]
        closef={}
        closeb={}
        expand_nodes =0
        U=100000000

        closef[start.state_hash()] = start
        heap.heappush(openf,start)
        closeb[goal.state_hash()] = goal
        heap.heappush(openb,goal)
        while len(openf)!=0 and len(openb)!=0:
            if U<=min(openf[0].get_cost(),openb[0].get_cost()):
                return U ,expand_nodes     
            if openf[0].get_cost()<openb[0].get_cost():
                n=heap.heappop(openf)
                for child in gridded_map.successors(n):
                    deltaX = abs(goal.get_x()-child.get_x())
                    deltaY = abs(goal.get_y()-child.get_y())

                    fv=child.get_g()+1.5*min(deltaX,deltaY)+abs(deltaX-deltaY)
                    hash_value = child.state_hash()
                    if hash_value in closeb:
                       U=min(U,closeb[hash_value].get_g()+child.get_g())
                      
                    if hash_value in closef and fv<closef[hash_value].get_cost():
                        child.set_cost(fv)
                        heap.heappush(openf,child)
                        closef[hash_value]=child
                       
                    if hash_value not in closef:
                        child.set_cost(fv)
                        heap.heappush(openf,child)
                        closef[hash_value]=child
                        expand_nodes+=1
            else:
                n=heap.heappop(openb)
                for child in gridded_map.successors(n):
                    deltaX = abs(start.get_x()-child.get_x())
                    deltaY = abs(start.get_y()-child.get_y())

                    fv=child.get_g()+1.5*min(deltaX,deltaY)+abs(deltaX-deltaY)
                    hash_value = child.state_hash()
                    if hash_value in closef:
                       U=min(U,closef[hash_value].get_g()+child.get_g())
                      
                    if hash_value in closeb and fv<closeb[hash_value].get_cost():
                        child.set_cost(fv)
                        heap.heappush(openb,child)
                        closeb[hash_value]=child
                       
                    if hash_value not in closeb:
                        child.set_cost(fv)
                        heap.heappush(openb,child)
                        closeb[hash_value]=child
                        expand_nodes = expand_nodes +1 
        return -1,len(closef) + len(closeb)

def MM(start,goal,map):
        openlist_fl=[]
        openlist_bl=[]
        closedict_fl={}
        closedict_bl={}
        U=1000000
        node = 0 
        
        heap.heappush(openlist_fl,start)
        heap.heappush(openlist_bl,goal)
        closedict_fl[start.state_hash()] = start
        closedict_bl[goal.state_hash()] = goal
       
       
        while len(openlist_fl)!=0 and len(openlist_bl)!=0:
          
            if U<=min(openlist_fl[0].get_cost(), openlist_bl[0].get_cost()):
                return U ,node
            if openlist_fl[0].get_cost()<openlist_bl[0].get_cost():
                n=heap.heappop(openlist_fl)
                for child in map.successors(n):
                    deltaX = abs(goal.get_x()-child.get_x())
                    deltaY = abs(goal.get_y()-child.get_y())

                    f_v=child.get_g()+1.5*min(deltaX,deltaY)+abs(deltaX-deltaY)
                    p_v=max(f_v,2*child.get_g())
                    hash_value = child.state_hash()
                    if hash_value in closedict_bl:
                       U=min(U,closedict_bl[hash_value].get_g()+child.get_g())
                      
                    if hash_value in closedict_fl and child.get_g()<closedict_fl[hash_value].get_g():
                        child.set_cost(p_v)
                        heap.heappush(openlist_fl,child)
                        closedict_fl[hash_value]=child
                     
                    if hash_value not in closedict_fl:
                        child.set_cost(p_v)
                        heap.heappush(openlist_fl,child)
                        closedict_fl[hash_value]=child
                       

            else:
                n=heap.heappop(openlist_bl)
                for child in map.successors(n):
                    deltaX =abs(start.get_x()-child.get_x())
                    deltaY = abs(start.get_y()-child.get_y())

                    f_v=child.get_g()+1.5*min(deltaX,deltaY)+abs(deltaX-deltaY)
                    
                    p_v=max(f_v,2*child.get_g())
                    hash_value = child.state_hash()
                    if hash_value in closedict_fl:
                       U=min(U,closedict_fl[hash_value].get_g()+child.get_g())
                      
                    if hash_value in closedict_bl and child.get_g()<closedict_bl[hash_value].get_g():
                        child.set_cost(p_v)
                        heap.heappush(openlist_bl,child)
                        closedict_bl[hash_value]=child
                      
                    if hash_value not in closedict_bl:
                        child.set_cost(p_v)
                        heap.heappush(openlist_bl,child)
                        closedict_bl[hash_value]=child
                        node = node + 1 
        return -1,node 



def main():
    """
    Function for testing your implementation. Run it with a -help option to see the options available. 
    """
    optlist, _ = getopt.getopt(sys.argv[1:], 'h:m:r:', ['testinstances', 'plots', 'help'])

    plots = False
    for o, _ in optlist:
        if o in ("-help"):
            print("Examples of Usage:")
            print("Solve set of test instances: main.py --testinstances")
            print("Solve set of test instances and generate plots: main.py --testinstances --plots")
            exit()
        elif o in ("--plots"):
            plots = True
    test_instances = "test-instances/testinstances.txt"
    gridded_map = Map("dao-map/brc000d.map")
    
    nodes_expanded_biastar = []   
    nodes_expanded_astar = []   
    nodes_expanded_mm = []
    
    start_states = []
    goal_states = []
    solution_costs = []
       
    file = open(test_instances, "r")
    for instance_string in file:
        list_instance = instance_string.split(",")
        start_states.append(State(int(list_instance[0]), int(list_instance[1])))
        goal_states.append(State(int(list_instance[2]), int(list_instance[3])))
        
        solution_costs.append(float(list_instance[4]))
    file.close()
        
    for i in range(0, len(start_states)):   

        start = start_states[i]
        goal = goal_states[i]
    
        cost, expanded_astar = None, None # Replace None, None with a call to your implementation of A*
        nodes_expanded_astar.append(expanded_astar)

        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by A* and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()

        cost, expanded_mm = None, None # Replace None, None with a call to your implementation of MM
        nodes_expanded_mm.append(expanded_mm)
        
        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by MM and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()

        cost, expanded_biastar = None, None # Replace None, None with a call to your implementation of Bi-A*
        nodes_expanded_biastar.append(expanded_biastar)
        
        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by Bi-A* and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()
    
    print('Finished running all tests. The implementation of an algorithm is likely correct if you do not see mismatch messages for it.')

    if plots:
        from search.plot_results import PlotResults
        plotter = PlotResults()
        plotter.plot_results(nodes_expanded_mm, nodes_expanded_astar, "Nodes Expanded (MM)", "Nodes Expanded (A*)", "nodes_expanded_mm_astar")
        plotter.plot_results(nodes_expanded_mm, nodes_expanded_biastar, "Nodes Expanded (MM)", "Nodes Expanded (Bi-A*)", "nodes_expanded_mm_biastar")
        

if __name__ == "__main__":
    main()