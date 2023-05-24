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


            if(tfinal - t < 50):
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

def metamorphic_test(num_trials):
    # kp = 16
    # ki = 0.5
    # kd = 2
    # Setpoint = 25 #20 and 50
    # System_status = 13
    Faulty = False
    result = {}
    agents = {}
    for i in range(0,num_trials):
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
        agent = random.randint(1,3)
        if agent == 1:
            j = 1
        elif agent == 2:
            j = 3
        else:
            j = 4
        if(j in agents):
                    agents[j] += 1
        else:
                    agents[j] = 1
        random_selection = j
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


        # if(Faulty):
        #     print("pid contrioller is Faulty!")
        # else:
        #     print("pid works well")
        print(f"------------------------------------  Iteration {i+1} is done -----------------------------------------------------")
    print('number of times that each agent has selected')
    print(agents)
    print('\n','-------------------------------------------------------------')
    print('number of times that each agent appeared in Faulty execution')
    print(result)
    #Entry = input("wait")
if __name__ == '__main__':
    #MR1(10,12)
    num_trials = 20
    metamorphic_test(num_trials)
    #PID_Runner(27,16,14,0.133,1.486)
