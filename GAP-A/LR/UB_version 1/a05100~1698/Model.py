from Read_data import Read_GAP_data
import copy
import cplex
class ADMM:
    def __init__(self):
        # self.multiplier=-10 #initial multiplier
        mod=Read_GAP_data()
        self.g_number_of_agents,self.g_number_of_jobs,self.agent_list,self.job_list=mod.read_data()
        # self.g_multipliers_list=[self.multiplier]*self.g_number_of_jobs
        self.g_iteration_times=100
        self.big_M=500
        #DP
        self.g_machine_state_vector_list=[[] for i in range (self.g_number_of_agents)]
        self.g_ending_state_vector_list=[None]*self.g_number_of_agents

        #results
        self.assignment_record=[] #the assignment matrix in each iteration
        self.serving_times=[] # record serving times of each job
        self.repeat_served=[] #repeat served in each iteration
        self.un_served=[] #unserved in each iteration
        self.record_multiplier=[] #multipliers in each iteration
        #LB,UB
        self.max_label_cost=float("inf")
        self.ADMM_local_LB = [0] *self.g_iteration_times
        self.ADMM_local_UB = [0] *self.g_iteration_times
        self.ADMM_global_LB = [-self.max_label_cost] *self.g_iteration_times
        self.ADMM_global_UB = [self.max_label_cost] *self.g_iteration_times


    def g_solving_the_GAP_by_LR_heuristics(self):
        for i in range(self.g_iteration_times):
            self.assignment_record.append([])
            self.serving_times.append([0]*self.g_number_of_jobs)
            self.repeat_served.append([])
            self.un_served.append([])
            self.record_multiplier.append([])

#step 1:UB generation
            # step 1.1 copy the multipliers
            for j in range(self.g_number_of_jobs):
                job=self.job_list[j]
                job.multiplier_ALR=copy.copy(job.multiplier)
            # step 1.2 solve the KS one by one
            for m in range(self.g_number_of_agents):
                self.g_solve_KS_of_ALR(m)
                served_jobs=self.Knapsack.solution.get_values()

                # calculate the primal cost handly
                primal_cost_of_agent_m=0
                Agent = self.agent_list[m]
                for j in range(self.g_number_of_jobs):
                    if served_jobs[j]==1:
                        self.job_list[j].multiplier_ALR = 100000
                        self.serving_times[i][j]+=1
                        primal_cost_of_agent_m+=Agent.cost_list_each_job[j]
                # self.ADMM_local_UB[i] +=primal_cost_of_agent_m
                self.assignment_record[i].append(served_jobs)

                """
                self.g_solve_KS(m,2)
                served_jobs=self.g_ending_state_vector_list[m].state_vector[0].served_jobs
                for j in served_jobs:
                    self.job_list[j].multiplier_ALR=100000
                    self.serving_times[i][j]+=1
                
                self.ADMM_local_UB[i]+=self.g_ending_state_vector_list[m].state_vector[0].primal_cost
                self.assignment_record[i].append(served_jobs)
                """
            self.assignment_matrix = []
            for j in range(self.g_number_of_agents):
                self.assignment_matrix.append([0] * self.g_number_of_jobs)
            S_0 = []
            S_1 = []
            for j in range(self.g_number_of_jobs):
                if self.serving_times[i][j]==1:
                    S_1.append(j)
                if self.serving_times[i][j]<1:
                    S_0.append(j) # re-assignment

            residual_capacity_list = []
            for m in range(self.g_number_of_agents):
                Agent = self.agent_list[m]
                residual_capacity_list.append(Agent.resource)
            # For S1:
            for j in S_1:
                for m in range(self.g_number_of_agents):
                    Agent = self.agent_list[m]
                    if self.assignment_record[i][m][j] == 1:
                        self.assignment_matrix[m][j] = 1  # record
                        self.ADMM_local_UB[i] += Agent.cost_list_each_job[j]  # S_0
                        residual_capacity_list[m] -= Agent.resource_list_each_job[j]  # update resource

            # For S0: unserved
            for j in S_0:
                served_KS_list = []
                for m in range(self.g_number_of_agents):
                    Agent = self.agent_list[m]
                    resouce_of_j = Agent.resource_list_each_job[j]
                    if residual_capacity_list[m] - resouce_of_j > 0:
                        served_KS_list.append(m)

                utility_list = []
                for m in served_KS_list:
                    Agent = self.agent_list[m]
                    utility = (Agent.cost_list_each_job[j]+self.job_list[j].multiplier)/Agent.resource_list_each_job[j]
                    utility_list.append(utility)
                # Find a KS (min)
                if utility_list == []:
                    self.ADMM_local_UB[i] += self.big_M
                    self.un_served[i].append(j)
                else:
                    min_utility = min(utility_list)
                    index = utility_list.index(min_utility)
                    m = served_KS_list[index]
                    Agent = self.agent_list[m]
                    self.assignment_matrix[m][j] = 1  # record
                    self.ADMM_local_UB[i] += Agent.cost_list_each_job[j]  # S_0
                    residual_capacity_list[m] -= Agent.resource_list_each_job[j]  # update resource

            self.assignment_record[i]=self.assignment_matrix


# step 2:LB generation
            serve_times=[0]*self.g_number_of_jobs
            for m in range(self.g_number_of_agents):#first term
                self.g_solve_KS_of_LR(m)
                self.ADMM_local_LB[i]+=self.Knapsack.solution.get_objective_value()
                served_jobs=self.Knapsack.solution.get_values()
                for j in range(self.g_number_of_jobs):
                    if served_jobs[j]==1:
                        serve_times[j]+=1

                """
                self.g_solve_KS(m,1)
                print(m)
                self.ADMM_local_LB[i]+=self.g_ending_state_vector_list[m].state_vector[0].cost_for_LR
                served_jobs=self.g_ending_state_vector_list[m].state_vector[0].served_jobs
                
                for j in served_jobs:
                    serve_times[j]+=1
                """

            for j in range(self.g_number_of_jobs):#multiplier term
                self.ADMM_local_LB[i]-=self.job_list[j].multiplier

# step 3:multiplier update
            #step size:
            # step 3.1 update gloabal UB/LB
            if i==0:
                self.ADMM_global_LB[i] = self.ADMM_local_LB[i]
                self.ADMM_global_UB[i] =self.ADMM_local_UB[i]
            else:
                self.ADMM_global_LB[i] = max(self.ADMM_local_LB[i],self.ADMM_global_LB[i-1])
                self.ADMM_global_UB[i] = min(self.ADMM_local_UB[i],self.ADMM_global_UB[i-1])

            print("iteration_{}_UB:{}".format(i,self.ADMM_global_UB[i]))
            print("iteration_{}_LB:{}".format(i,self.ADMM_global_LB[i]))

            # step size-1
            # step_size = 10 / (i + 1)
            # if step_size < 0.1:
            #     step_size = 0.1

            # step size-2
            difference=0
            for j in range(self.g_number_of_jobs):
                difference+=(serve_times[j]-1)**2
            if difference==0:
                step_size=0
            else:
                step_size=(self.ADMM_global_UB[i]-self.ADMM_global_LB[i])/difference

            # step size-3
            # step_size=1
            for j in range(self.g_number_of_jobs):
                #record the multipliers
                self.record_multiplier[i].append(self.job_list[j].multiplier)
                self.job_list[j].multiplier+=step_size*(serve_times[j]-1)
#Step 4: Terminal condition
#Note: for min 1, for max -1
            if self.ADMM_global_LB[i]!=0:
                gap=(self.ADMM_global_UB[i]-self.ADMM_global_LB[i])/self.ADMM_global_UB[i]
            else:
                gap=1
            if gap<0.005:
                # self.g_iteration_times_total=i+1
                break
        # self.g_iteration_times_total=self.g_iteration_times

    def g_solve_KS_of_LR(self,m):
        self.Knapsack = cplex.Cplex()
        Agent=self.agent_list[m]
        cost_list_each_job=Agent.cost_list_each_job
        #obj
        for j in range(self.g_number_of_jobs):
            job = self.job_list[j]
            multiplier=job.multiplier
            cost=multiplier+cost_list_each_job[j]
            self.Knapsack.variables.add(obj=[cost],types=[self.Knapsack.variables.type.binary],names=["x_{}".format(j)])
        #capacity (resource)
        capacity=Agent.resource
        lin_express=[[],Agent.resource_list_each_job]
        for j in range(self.g_number_of_jobs):
            lin_express[0].append("x_{}".format(j))
        self.Knapsack.linear_constraints.add(lin_expr=[lin_express],senses=["L"],rhs=[capacity])
        self.Knapsack.set_results_stream(None)
        self.Knapsack.solve()

    def g_solve_KS_of_ALR(self,m):
        self.Knapsack = cplex.Cplex()
        Agent = self.agent_list[m]
        cost_list_each_job = Agent.cost_list_each_job
        # obj
        for j in range(self.g_number_of_jobs):
            job = self.job_list[j]
            multiplier = job.multiplier_ALR
            cost = multiplier + cost_list_each_job[j]
            self.Knapsack.variables.add(obj=[cost], types=[self.Knapsack.variables.type.binary],names=["x_{}".format(j)])
        # capacity (resource)
        capacity = Agent.resource
        lin_express = [[], Agent.resource_list_each_job]
        for j in range(self.g_number_of_jobs):
            lin_express[0].append("x_{}".format(j))
        self.Knapsack.linear_constraints.add(lin_expr=[lin_express], senses=["L"], rhs=[capacity])
        self.Knapsack.set_results_stream(None)
        self.Knapsack.solve()


    def g_output_the_results(self,spend_time):
        #Assignment results
        with open("Assignment_result.csv","w") as fl:
            fl.write("iteration,agent,jobs\n")
            for i in range(len(self.assignment_record)):
                for m in range(self.g_number_of_agents):
                    result_for_m=self.assignment_record[i][m]
                    str_=""
                    for j in result_for_m:
                        str_=str_+"_"+str(j)
                    fl.write(str(i)+","+str(m)+","+str_+"\n")
        with open("gap.csv","w") as fl:
            iterations=len(self.assignment_record)
            fl.write("iteration,local_LB,local_UB,LB,UB,gap,un_served,repeat_served\n")
            for i in range(iterations):
                local_UB = round(self.ADMM_local_UB[i], 3)
                local_LB = round(self.ADMM_local_LB[i], 3)
                UB=round(self.ADMM_global_UB[i],3)
                LB=round(self.ADMM_global_LB[i],3)
                if UB!=0:
#Note
                    gap=round((UB-LB)/UB,3)
                str1=""
                str2=""
                for j in self.un_served[i]:
                    str1=str1+"_"+str(j)
                for j in self.repeat_served[i]:
                    str2=str2+"_"+str(j)
                fl.write(str(i)+","+str(local_LB)+","+str(local_UB)+","+str(LB)+","+str(UB)+","+str(gap)+","+str1+","+str2+"\n")
            fl.write("Running time: {} sec".format(round(spend_time,2)))

        with open("Multiplier.csv","w") as fl:
            fl.write("iteration")
            for j in range(self.g_number_of_jobs):
                fl.write(","+str(j))
            fl.write("\n")
            for i in range(iterations):
                fl.write(str(i))
                multiplier_list=self.record_multiplier[i]
                for j in range(self.g_number_of_jobs):
                    multiplier=round(multiplier_list[j],2)
                    fl.write(","+str(multiplier))
                fl.write("\n")

    def g_solve_KS(self,m,Flag):
        #initialization
        self.g_machine_state_vector_list[m]=[]
        for j_ in range(self.g_number_of_jobs+1):
            stage=State_vector()
            stage.stage_id=j_
            stage.state_vector=[]
            self.g_machine_state_vector_list[m].append(stage)

        self.g_ending_state_vector_list[m]=State_vector()
        self.g_ending_state_vector_list[m].Reset()
        state=State()
        self.g_machine_state_vector_list[m][0].state_vector.append(state) #0: border state

        #loop 1: for each job
        for j in range(self.g_number_of_jobs):
            state_list=self.g_machine_state_vector_list[m][j].state_vector
            for element in state_list:
                for s in range(0,2):#装不装j，0 no， 1yes
                    # new state
                    new_element= State()
                    new_element.copy(element)
                    #case-1: 不装
                    if s==0:
                        #ending stage vector
                        if j==self.g_number_of_jobs-1:
                            self.g_ending_state_vector_list[m].update_stage_state(new_element,Flag)
                        self.g_machine_state_vector_list[m][j+1].update_stage_state(new_element,Flag)
                        continue
                    #case-2:装
                    if s==1:
                        if element.weight+self.agent_list[m].resource_list_each_job[j]>self.agent_list[m].resource:
                            continue
                        else:
                            new_element.weight+=self.agent_list[m].resource_list_each_job[j]
                            new_element.served_jobs.append(j)
                            new_element.calculate_cost(self.agent_list[m],self.job_list[j])
                            self.g_machine_state_vector_list[m][j+1].update_stage_state(new_element, Flag)
                            if j == self.g_number_of_jobs-1:
                                self.g_ending_state_vector_list[m].update_stage_state(new_element, Flag)
                            continue
        self.g_ending_state_vector_list[m].sort(Flag) #Note: min
        # print()


class State_vector:
    def __init__(self):
        self.state_vector_id=0
        self.state_vector=[]        #possible states in stage k

    def Reset(self):
        self.state_vector_id=0
        self.state_vector = []  # possible states in stage k

    def update_stage_state(self,element,Flag):
        weight=element.weight
        if Flag==1:#LR
            cost=element.cost_for_LR
            #when we cannot find a better state than element, we add the element.
#Note: add the new element?
            w=0  #1,存在一个比element更优的；0，不存在
            for state in self.state_vector:
                cost0=state.cost_for_LR
                weight0=state.weight
                if cost0<cost and weight0<=weight: #exist a state
                    w=1
#Note: delete an existed state?
                if cost0>cost and weight0>=weight:
                    self.state_vector.remove(state)
            #add the element
            if w==0:
                self.state_vector.append(element)

        if Flag == 2:  # ALR
            cost = element.cost_for_ALR
            # when we cannot find a better state than element, we add the element.
            w = 0  # 1,存在一个比element更优的；0，不存在
            for state in self.state_vector:
                cost0 = state.cost_for_ALR
                weight0 = state.weight
                if cost0 < cost and weight0 <= weight:  # exist a state
                    w = 1
                # Note: delete an existed state?
                if cost0 > cost and weight0 >= weight:
                    self.state_vector.remove(state)
            # add the element
            if w == 0:
                self.state_vector.append(element)


    def sort(self,Flag):
        if Flag == 1:
            self.state_vector = sorted(self.state_vector, key=lambda x: x.cost_for_LR)
        if Flag == 2:
            self.state_vector = sorted(self.state_vector, key=lambda x: x.cost_for_ALR)

class State:
    def __init__(self):
        self.weight=0
        self.served_jobs=[]
        self.primal_cost=0
        self.cost_for_LR=0
        self.cost_for_ALR=0
    def copy(self,element):
        self.weight=copy.copy(element.weight)
        self.served_jobs = []
        self.served_jobs = copy.copy(element.served_jobs)
        self.primal_cost = copy.copy(element.primal_cost)
        self.cost_for_LR = copy.copy(element.cost_for_LR)
        self.cost_for_ALR = copy.copy(element.cost_for_ALR)

    def calculate_cost(self,agent,job):
        #agent: current machine; job：current job.(class)
        job_id=job.job_id
        self.primal_cost-=agent.cost_list_each_job[job_id]
        self.cost_for_LR=self.cost_for_LR-agent.cost_list_each_job[job_id]+job.multiplier
        self.cost_for_ALR=self.cost_for_ALR-agent.cost_list_each_job[job_id]+job.multiplier_ALR
#note: "-" for max problem; "+" min problem