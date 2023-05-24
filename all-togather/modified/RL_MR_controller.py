import operator
import numpy
import random
from tclab import clock, setup, Historian, Plotter
def PID(Kp, Ki, Kd, MV_bar=0, beta=1, gamma=0):
    # initialize stored data
    eD_prev = 0
    t_prev = -100
    I = 0

    # initial control
    MV = MV_bar

    while True:
        # yield MV, wait for new t, SP, PV
        t, PV, SP = yield MV

        # PID calculations
        P = Kp*(beta*SP - PV)
        #print(t,P,SP,PV)
        I = I + Ki*(SP - PV)*(t - t_prev)
        eD = gamma*SP - PV
        D = Kd*(eD - eD_prev)/(t - t_prev)
        MV = MV_bar + P + I + D

        # update stored data for next iteration
        eD_prev = eD
        t_prev = t


def PID_Runner(Setpoint,System_status,kp,ki,kd):
    TCLab = setup(connected=False, speedup=10)

    controller = PID(kp, ki, kd, beta=1)   # create pid control
    controller.send(None)                 # initialize

    tfinal = 1200

    with TCLab() as lab:
        lab.Ta = System_status
        #T1 = 0
        T1 = lab.Ta
        lab.T1_setter(T1)
        #print(lab.T1 , lab.Ta)
        #h = Historian([('SP', lambda: SP), ('PV', lambda: lab.T1), ('MV', lambda: MV), ('Q1', lab.Q1)])
        h = Historian([('SP', lambda: SP), ('PV', lambda: lab.T1), ('MV', lambda: MV)])
        p = Plotter(h, tfinal)

        diff = 0
        rise_diff_start = 1
        rise_diff_stop = 1
        settle_diff = 1

        lst = {}
        settle_time_start = numpy.inf
        setpoint = Setpoint
        dict={}
        stlle = {}

        for t in clock(tfinal, 2):

            #SP = T1 if t < 50 or (t > 500 and t < 750) else 50           # get setpoint
            SP = T1 if t < 50 else setpoint
            #SP = setpoint if t < 50 else setpoint
            PV = lab.T1                         # get measurement

            if(PV - SP > diff):
                diff = PV - SP
                PV_fin = PV
                SP_fin = SP
                T_fin = t
                #print(T_fin)

            MV = controller.send([t, PV, SP])   # compute manipulated variable
            #print(MV,t)

            lab.U1 = MV                         # apply
            p.update(t)                         # update information display


            if(tfinal - t < 200):
                if(t in dict):
                    dict[PV] +=1
                else:
                    dict[PV] = 1



            if(abs(round(PV,1) - (0.1*(setpoint-T1)+T1)) < rise_diff_start):
                rise_start = t
                rise_diff_start = abs(round(PV,1) - (0.1*(setpoint-T1)+T1))
                rise_start_value = PV

            elif(abs(round(PV,1) - (0.9*(setpoint-T1)+T1)) < rise_diff_stop):

                rise_stop = t
                rise_diff_stop = abs(round(PV,1) - (0.9*(setpoint-T1)+T1))
                rise_stop_value = PV
                lst[rise_stop] = rise_stop_value
                #print(rise_stop)

            else:
                pass

            if(PV > T1 and t<settle_time_start):
                settle_time_start = t
            if(abs(round(PV,1) - (0.98*setpoint)) < settle_diff):
                settle_time_stop = t
                diff_dict = abs(round(PV,1) - (0.98*setpoint))
                stlle[diff_dict] = settle_time_stop
            elif(abs(round(PV,1) - (1.02*setpoint)) < settle_diff):
                settle_time_stop = t
                diff_dict = abs(round(PV,1) - (1.02*setpoint))
                stlle[diff_dict] = settle_time_stop
            else:
                pass
        stable = max(dict.items(), key=operator.itemgetter(1))[0]
        try:
            percentage_overshhot = diff + (SP - stable)
            print("percentage overshhot is = ", percentage_overshhot)
        except:
            pass
        try:
            print(f"for PV = {PV_fin} and SP = {SP_fin} ======> peak Time is = {T_fin}")
        except:
            pass
        try:
            print("final value is : " , stable)
        except:
            pass
        try:
            steady_state_error = abs(SP - stable)
            print("steady state error is :",  steady_state_error)
        except:
            pass
        for key in list(lst):
            if(lst[key] > T_fin):
                del lst[key]
        rise_stop = sorted(lst.keys())[-1]
        rise_stop_value = lst[rise_stop]

        try:
            rise_time = rise_stop - rise_start
            print("rise time is : ",rise_time, f"rise start time is {rise_start} with value {rise_start_value} and rise stop time is {rise_stop} with value {rise_stop_value} ")
        except:
            pass

        for key in list(stlle):
            if(stlle[key] <= T_fin):
                del stlle[key]
        settle_timestop_value = sorted(stlle.keys())[0]
        settle_time_stop = stlle[settle_timestop_value]
        try:
            settle_time = settle_time_stop - settle_time_start
            print("settling time is : ",settle_time,settle_time_stop,settle_time_start)
        except:
            pass

        print("\n\n")
        try:
            print("Finally we have these values: ")
            print("------------------------------------------------------------------------")
            print(f"|       Settling Time            |             {round(settle_time_stop - settle_time_start,2)}                  ")
            print(f"|       Peak Time                |             {round(T_fin,2)}                   ")
            print(f"|       Rise Time                |             {round(rise_stop - rise_start,2)}                    ")
            print(f"|       Percentage Overshoot     |             {round(diff + (SP - stable),2)}                    ")
            print(f"|       Steady state error       |             {round(abs(SP - stable),2)}                    ")
            print("------------------------------------------------------------------------")
        except:
            pass

    #Entry = input("wait")
    return settle_time,T_fin,rise_time,percentage_overshhot,steady_state_error



def MR1(Setpoint_Source , System_status_Source):

    Source_difference = Setpoint_Source - System_status_Source
    follow_difference = -1
    Setpoint_follow = 0
    Systemstatus_follow = 0
    while (follow_difference <= Source_difference):
        Setpoint_follow = random.randrange(20,30)
        Systemstatus_follow = random.randrange(8,18)
        follow_difference = Setpoint_follow - Systemstatus_follow

    Kp = random.randrange(follow_difference+2,follow_difference+5)
    Ki = round(random.uniform(0.1,0.2),2)
    Kd = round(random.uniform(1.2,3.0),2)



    return Setpoint_follow,Systemstatus_follow,Kp,Ki,Kd




def MR2(Kp):

    Setpoint_follow = random.randrange(20,30)
    Systemstatus_follow = random.randrange(8,18)
    follow_difference = Setpoint_follow - Systemstatus_follow
    Kp_follow = random.randrange(Kp+5+follow_difference,Kp+8+follow_difference)
    while follow_difference >= Kp_follow and Kp_follow >= follow_difference * 2:
        Setpoint_follow = random.randrange(20,30)
        Systemstatus_follow = random.randrange(8,18)
        follow_difference = Setpoint_follow - Systemstatus_follow
    # while Kp_follow >= follow_difference * 2:
    #     Kp_follow = random.randrange(Kp+5+follow_difference,Kp+8+follow_difference)
    Ki = 0
    Kd = 0


    return Setpoint_follow,Systemstatus_follow,Kp_follow,Ki,Kd




def MR3(Ki):
    Setpoint_follow = random.randrange(20,30)
    Systemstatus_follow = random.randrange(8,18)
    while(Setpoint_follow<=Systemstatus_follow):
        Setpoint_follow = random.randrange(20,30)
        Systemstatus_follow = random.randrange(8,18)
    MR_diff = Setpoint_follow - Systemstatus_follow
    Kp = random.randrange(MR_diff+2,MR_diff+5)
    Ki_follow = -1
    while Ki > Ki_follow:
        Ki_follow = round(random.uniform(0.1,0.2),2)
    Kd = round(random.uniform(1.2,3.0),2)


    return Setpoint_follow,Systemstatus_follow,Kp,Ki_follow,Kd

def MR4(Kd):
    Setpoint_follow = random.randrange(20,30)
    #Setpoint_follow = 30
    Systemstatus_follow = random.randrange(8,18)
    #Systemstatus_follow = 0
    while(Setpoint_follow<=Systemstatus_follow):
        Setpoint_follow = random.randrange(20,30)
        Systemstatus_follow = random.randrange(8,18)

    MR_diff = Setpoint_follow - Systemstatus_follow
    Kp = random.randrange(MR_diff+2,MR_diff+5)
    Ki = round(random.uniform(0.1,0.2),2)
    Kd_follow = -1
    while Kd > Kd_follow:
        Kd_follow = round(random.uniform(1.2,3.0),2)



    return Setpoint_follow,Systemstatus_follow,Kp,Ki,Kd_follow



import matplotlib.pyplot as plt
import numpy as np


class Bandit:
    def __init__(self,p):
        # self.mean = mean
        # self.variance = 2
        self.p = p
        self.p_estimate = 0
        self.n = 0

    def pull(self,result):
        # return np.random.random() < self.p
        return result

    def update(self,x):
        self.n +=1
        self.p_estimate = ((self.n - 1)* self.p_estimate + x) / self.n

def check_for_eps(step,eps):
    if(step>0 and step%2==0):
        return eps - 0.01
    else:
        return eps

def experiment(num_trials,eps,bandit_probs):
    bandits = [Bandit(p) for p in bandit_probs]
    rewards = np.zeros(num_trials)
    num_times_explored = 0
    num_times_exploited = 0
    num_optimal = 0
    optimal_j = np.argmax([bandit.p for bandit in bandits])
    print("optimal j is : ", optimal_j)
    for item in range(num_trials):
        if (np.random.random() < eps):
            num_times_explored+=1
            j = np.random.randint(len(bandits))
        else:
            num_times_exploited+=1
            j = np.argmax([b.p_estimate for b in bandits])
        if(j == optimal_j):
            num_optimal+=1
        x = bandits[j].pull()
        rewards[item] = x
        bandits[j].update(x)
        eps = check_for_eps(item,eps)

    for b in bandits:
        print("mean estimate (bandit-p_estimate) : ",b.p_estimate)
    print("total reward earned", rewards.sum())
    print("Overal win rate : ", rewards.sum()/num_trials)
    print("num times explored : " , num_times_explored)
    print("num times exploited : " , num_times_exploited)
    print("num times optimal bandit selected : " , num_optimal)
    cumulative_reward = np.cumsum(rewards)
    win_rate = cumulative_reward / (np.arange(num_trials)+1)
    plt.plot(win_rate)
    plt.plot(np.ones(num_trials)*np.max(bandit_probs))
    plt.show()

def metamorphic_test(num_trials,eps,bandit_probs):
    # kp = 16
    # ki = 0.5
    # kd = 2
    # Setpoint = 25 #20 and 50
    # System_status = 13



    bandits = [Bandit(p) for p in bandit_probs]
    rewards = np.zeros(num_trials)
    num_times_explored = 0
    num_times_exploited = 0
    num_optimal = 0
    optimal_j = np.argmax([bandit.p for bandit in bandits])


    result = {}
    agents = {}
    for i in range(0,num_trials):
        Faulty = False
        Setpoint_Source = random.randrange(20,30)
        Systemstatus_Source = random.randrange(8,18)
        while(Setpoint_Source<=Systemstatus_Source):
            Setpoint_Source = random.randrange(20,30)
            Systemstatus_Source = random.randrange(8,18)
        Source_difeerence = Setpoint_Source - Systemstatus_Source
        # Kp = random.randrange(Source_difeerence+2,Source_difeerence+5)
        # print(Setpoint_Source,Systemstatus_Source,Kp)
        Kp = random.randrange(Source_difeerence+1,Source_difeerence+4)
        Ki = round(random.uniform(0.1,0.2),2)
        Kd = round(random.uniform(1.2,3.0),2)
        # Ki = 0
        # Kd = 0
        ts1,tp1,tr1,os1,ess1 = PID_Runner(Setpoint_Source,Systemstatus_Source,Kp,Ki,Kd)

        print("End of Source")
        print("in Source we have : ",Setpoint_Source,Systemstatus_Source,Kp,Ki,Kd)
        print("Follow-up started")
        # random_selection = random.randint(1,5)
        ran1 = np.random.random()
        print('test11',ran1,eps)
        if ( ran1 < eps):
            num_times_explored+=1
            j = np.random.randint(len(bandits))
        else:
            num_times_exploited+=1
            j = np.argmax([b.p_estimate for b in bandits])
        # if(j == optimal_j):
        #     num_optimal+=1
        if j == 0:
            j = 0
        elif j == 1:
            j = 2
        else:
            j = 3
        if(j in agents):
                    agents[j] += 1
        else:
                    agents[j] = 1
        random_selection = j+1

        #########################################
        if(random_selection == 1):
        #MR1_SP,MR1_SS,MR1_Kp,MR1_Ki,MR1_Kd = MR1(Setpoint_Source,Systemstatus_Source)
            MR1_SP,MR1_SS,MR1_Kp,MR1_Ki,MR1_Kd = MR1(Setpoint_Source,Systemstatus_Source)

            print("FOLLOW_UP VALUES MR1 : ",MR1_SP,MR1_SS,MR1_Kp,MR1_Ki,MR1_Kd)

            ts2,tp2,tr2,os2,ess2 = PID_Runner(MR1_SP,MR1_SS,MR1_Kp,MR1_Ki,MR1_Kd)
            if(ts1<ts2):
                print("no Fault detected by MR1")

            else:
                print("there is a Fault by MR1")
                if("MR1" in result):
                    result["MR1"] += 1
                else:
                    result["MR1"] = 1
                Faulty = True
        # elif(random_selection == 2):
        #     MR2_SP,MR2_SS,MR2_Kp,MR2_Ki,MR2_Kd = MR2(Kp)
        #     print("FOLLOW_UP VALUES MR2 : ",MR2_SP,MR2_SS,MR2_Kp,MR2_Ki,MR2_Kd)
        #     ts2,tp2,tr2,os2,ess2 = PID_Runner(MR2_SP,MR2_SS,MR2_Kp,MR2_Ki,MR2_Kd)
        #
        #     if(tr1>=tr2 and os1<os2 and ess1>=ess2):
        #         print("no Fault detected by MR2")
        #
        #     else:
        #         print("there is a Fault by MR2")
        #         if("MR2" in result):
        #             result["MR2"] += 1
        #         else:
        #             result["MR2"] = 1
        #         Faulty = True
        elif(random_selection == 3):
            MR3_SP,MR3_SS,MR3_Kp,MR3_Ki,MR3_Kd = MR3(Ki)
            print("FOLLOW_UP VALUES MR3 : ",MR3_SP,MR3_SS,MR3_Kp,MR3_Ki,MR3_Kd)
            ts2,tp2,tr2,os2,ess2 = PID_Runner(MR3_SP,MR3_SS,MR3_Kp,MR3_Ki,MR3_Kd)
            if(os1<os2 and ess1>=ess2 and ts1 <= ts2):
                print("no Fault detected by MR3")

            else:
                print("there is a Fault by MR3")
                if("MR3" in result):
                    result["MR3"] += 1
                else:
                    result["MR3"] = 1
                Faulty = True
        else:
            MR4_SP,MR4_SS,MR4_Kp,MR4_Ki,MR4_Kd = MR4(Kd)
            print("FOLLOW_UP VALUES MR4 : ",MR4_SP,MR4_SS,MR4_Kp,MR4_Ki,MR4_Kd)
            ts2,tp2,tr2,os2,ess2 = PID_Runner(MR4_SP,MR4_SS,MR4_Kp,MR4_Ki,MR4_Kd)
            if(os1>=os2 and ess1>=ess2 and ts1 >= ts2):
                print("no Fault detected by MR4")

            else:
                print("there is a Fault by MR4")
                if("MR4" in result):
                    result["MR4"] += 1
                else:
                    result["MR4"] = 1
                Faulty = True

        if(Faulty == True):
            x = bandits[j].pull(1)
        else:
            x = bandits[j].pull(0)
        rewards[i] = x
        print('rew is:',rewards[i])
        bandits[j].update(x)
        eps = check_for_eps(i,eps)


        # if(Faulty):
        #     print("pid contrioller is Faulty!")
        # else:
        #     print("pid works well")
        print(f"------------------------------------  Iteration {i+1} was done -----------------------------------------------------")
    for item in range(len(bandits)):
        print(f'agnet{item} probability estimation: ',bandits[item].p_estimate)
    print('\n','-------------------------------------------------------------')
    print('number of exploration is: ',num_times_explored,'number of exploitation is: ',num_times_exploited)
    print('\n','-------------------------------------------------------------')
    # print(rewards)
    print('number of times that each agent has selected')
    print(agents)
    print('\n','-------------------------------------------------------------')
    print('number of times that each agent appeared in Faulty execution')
    print(result)

    #Entry = input("wait")


def main():
    num_trials = 20
    eps = 0.40
    bandit_prob = [0.25,0.25,0.25,0.25]
    metamorphic_test(num_trials,eps,bandit_prob)

# if  __name__ == '__main__':
#     main()


if __name__ == '__main__':
    #MR1(10,12)
    # metamorphic_test()
    #PID_Runner(27,16,14,0.133,1.486)
    main()
