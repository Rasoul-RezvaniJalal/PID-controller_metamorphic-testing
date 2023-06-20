import operator

import matplotlib.pyplot as plt
import numpy
import random
from tclab import clock, setup, Historian, Plotter


# def PID(Kp, Ki, Kd, MV_bar):
#     # initialize stored data
#     e_prev = 0
#     t_prev = -100
#     I = 0
#
#     # initial control
#     MV = MV_bar
#
#     while True:
#         # yield MV, wait for new t, PV, SP
#         t, PV, SP = yield MV
#
#         # PID calculations
#         e = SP - PV
#
#         P = Kp * e
#         I = I + Ki * e * (t - t_prev)
#         D = Kd * (e - e_prev) / (t - t_prev)
#
#         MV = MV_bar + P + I + D
#
#         # update stored data for next iteration
#         e_prev = e
#         t_prev = t

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

def PID_Runner(Setpoint, System_status, kp, ki, kd):
    TCLab = setup(connected=False, speedup=10)

    controller = PID(kp, ki, kd,MV_bar=System_status)  # create pid control
    controller.send(None)  # initialize

    tfinal = 1200

    with TCLab() as lab:
        lab.Ta = System_status
        # T1 = 0
        T1 = lab.Ta
        lab.T1_setter(T1)
        # print(lab.T1 , lab.Ta)
        # h = Historian([('SP', lambda: SP), ('PV', lambda: lab.T1), ('MV', lambda: MV), ('Q1', lab.Q1)])
        h = Historian([('SP', lambda: SP), ('PV', lambda: lab.T1), ('MV', lambda: MV)])
        p = Plotter(h, tfinal)

        # diff = 0
        # rise_diff_start = 1
        # rise_diff_stop = 1
        # settle_diff = 1
        #
        # lst = {}
        # settle_time_start = numpy.inf
        setpoint = Setpoint
        calculation = {}
        # dict={}
        # stlle = {}

        for t in clock(tfinal, 2):
            # SP = T1 if t < 50 or (t > 500 and t < 750) else 50           # get setpoint
            SP = T1 if t < 50 else setpoint
            # SP = setpoint if t < 50 else setpoint
            PV = lab.T1  # get measurement

            # if(PV - SP > diff):
            #     diff = PV - SP
            #     PV_fin = PV
            #     SP_fin = SP
            #     T_fin = t
            #     #print(T_fin)

            MV = controller.send([t, PV, SP])  # compute manipulated variable
            # print(PV,t)
            calculation[int(t)] = PV

            lab.U1 = MV  # apply
            p.update(t)  # update information display

        # print(calculation)
        peak_value = max(calculation.values())
        peak_time = list(calculation.keys())[list(calculation.values()).index(max(calculation.values()))]
        key_list = [key for key in calculation]
        common_values = {}
        # print(len(calculation)*3/4,len(calculation))
        for key in range(round(len(calculation) * 3 / 4), len(calculation)):
            if calculation[key_list[key]] in common_values:
                common_values[calculation[key_list[key]]] += 1
            else:
                common_values[calculation[key_list[key]]] = 1

        # print(common_values)
        steady_state_value = list(common_values.keys())[list(common_values.values()).index(max(common_values.values()))]
        steady_state_error = abs(setpoint - steady_state_value)
        # print(steady_state_error)
        # print(peak_time)
        percentage_overshoot = calculation[peak_time] - steady_state_value
        # print(percentage_overshoot)
        # print(calculation)
        before_rise = calculation[key_list[24]]
        for key in range(25, len(calculation)):
            if calculation[key_list[key]] > System_status and calculation[key_list[key]] != before_rise:
                growup_time = key_list[key]
                break
        # print(growup_time)
        # for key in (calculation[growup_time],calculation[peak_time]):
        #     # print(key)
        rise_diff_start = 1
        rise_diff_stop = 1
        # settle_diff = 1
        for key in range(len(calculation)):
            if growup_time < key_list[key] < peak_time:
                if abs((calculation[key_list[key]]) - (
                        0.1 * (steady_state_value - System_status) + System_status)) < rise_diff_start:
                    rise_start_time = key_list[key]
                    rise_diff_start = abs(
                        (calculation[key_list[key]]) - (0.1 * (steady_state_value - System_status) + System_status))
                    rise_start_value = calculation[key_list[key]]

                if abs((calculation[key_list[key]]) - (
                        0.9 * (steady_state_value - System_status) + System_status)) < rise_diff_stop:
                    rise_stop_time = key_list[key]
                    rise_diff_stop = abs(
                        (calculation[key_list[key]]) - (0.9 * (steady_state_value - System_status) + System_status))
                    rise_stop_value = calculation[key_list[key]]
                    # lst[rise_stop] = rise_stop_value
        # print(rise_start_value, rise_start_time)
        # print(rise_stop_value, rise_stop_time)
        flag = False
        for key in range(len(calculation)):
            # if key_list[key] > peak_time:
                if (1.02 * steady_state_value) > (calculation[key_list[key]]) > (0.98 * steady_state_value) and not flag:
                    settle_stop = key_list[key]
                    # print(settle_stop)
                    flag = True
                elif (1.02 * steady_state_value) < (calculation[key_list[key]]) or (calculation[key_list[key]]) < (0.98 * steady_state_value):
                    flag = False
        # print(settle_stop)
        settle_time = settle_stop - growup_time
        rise_time = rise_stop_time - rise_start_time

        try:
            print("Finally we have these values: ")
            print("------------------------------------------------------------------------")
            print(f"|       Settling Time            |             {settle_time}                  ")
            print(f"|       Peak Time                |             {peak_time}                   ")
            print(f"|       Rise Time                |             {rise_time}                    ")
            print(f"|       Percentage Overshoot     |             {percentage_overshoot}                    ")
            print(f"|       Steady state error       |             {steady_state_error}                    ")
            print("------------------------------------------------------------------------")
        except:
            pass

        return settle_time, peak_time, rise_time, percentage_overshoot, steady_state_error


def MR1(Setpoint_Source, System_status_Source):
    Source_difference = Setpoint_Source - System_status_Source
    follow_difference = -1
    Setpoint_follow = 0
    Systemstatus_follow = 0
    while follow_difference <= Source_difference:
        Setpoint_follow = random.randrange(2, 10)
        Systemstatus_follow = random.randrange(2, 10)
        follow_difference = Setpoint_follow - Systemstatus_follow

    Kp = random.uniform(follow_difference + 2, follow_difference + 10)
    Ki = random.uniform(0.002, 0.01)
    Kd = random.uniform(0.5, 10)

    return Setpoint_follow, Systemstatus_follow, Kp, Ki, Kd


def MR2(Kp):
    Setpoint_follow = random.randrange(2, 10)
    Systemstatus_follow = random.randrange(2, 10)
    while Setpoint_follow <= Systemstatus_follow:
            Setpoint_follow = random.randrange(2, 10)
            Systemstatus_follow = random.randrange(2, 10)
    # follow_difference = Setpoint_follow - Systemstatus_follow
    # Kp_follow = random.uniform(Kp + 2 + follow_difference, Kp + 10 + follow_difference)
    Kp_follow = random.uniform(2, 10)
    while Kp_follow <= Kp:
            Kp_follow = random.uniform(2, 10)

    # while follow_difference >= Kp_follow or Kp_follow >= follow_difference * 2:
    #     Setpoint_follow = random.randrange(2, 10)
    #     Systemstatus_follow = random.randrange(2, 10)
    #     follow_difference = Setpoint_follow - Systemstatus_follow
    # while Kp_follow >= follow_difference * 2:
    #     Kp_follow = random.randrange(Kp+5+follow_difference,Kp+8+follow_difference)
    Ki = 0
    Kd = 0

    return Setpoint_follow, Systemstatus_follow, Kp_follow, Ki, Kd


def MR3(Ki):
    Setpoint_follow = random.randrange(2, 10)
    Systemstatus_follow = random.randrange(2, 10)
    while Setpoint_follow <= Systemstatus_follow:
        Setpoint_follow = random.randrange(2, 10)
        Systemstatus_follow = random.randrange(2, 10)
    # MR_diff = Setpoint_follow - Systemstatus_follow
    Kp = random.uniform(2, 10)
    Ki_follow = -1
    while Ki > Ki_follow:
        Ki_follow = random.uniform(0.002, 0.01)
    Kd = random.uniform(0.5, 10)

    return Setpoint_follow, Systemstatus_follow, Kp, Ki_follow, Kd


def MR4(Kd):
    Setpoint_follow = random.randrange(2, 10)
    # Setpoint_follow = 30
    Systemstatus_follow = random.randrange(2, 10)
    # Systemstatus_follow = 0
    while (Setpoint_follow <= Systemstatus_follow):
        Setpoint_follow = random.randrange(2, 10)
        Systemstatus_follow = random.randrange(2, 10)

    MR_diff = Setpoint_follow - Systemstatus_follow
    Kp = random.uniform(2, 10)
    Ki = random.uniform(0.002, 0.01)
    Kd_follow = -1
    while Kd > Kd_follow:
        Kd_follow = random.uniform(0.5, 10)

    return Setpoint_follow, Systemstatus_follow, Kp, Ki, Kd_follow


def metamorphic_test(num_trials):
    # kp = 16
    # ki = 0.5
    # kd = 2
    # Setpoint = 25 #20 and 50
    # System_status = 13
    faulty = False
    result = {}
    agents = {}
    for i in range(0, num_trials):
        Setpoint_Source = random.randrange(1, 8)
        Systemstatus_Source = random.randrange(1, 8)
        while Setpoint_Source <= Systemstatus_Source:
            Setpoint_Source = random.randrange(1, 8)
            Systemstatus_Source = random.randrange(1, 8)
        Source_difeerence = Setpoint_Source - Systemstatus_Source
        # # Kp = random.randrange(Source_difeerence+2,Source_difeerence+5)
        # # print(Setpoint_Source,Systemstatus_Source,Kp)
        # Kp = random.randrange(Source_difeerence + 1, Source_difeerence + 4)
        Kp = random.uniform(1, 8)
        Ki = random.uniform(0.001, 0.006)
        Kd = random.uniform(0, 8)
        # print(Kp,Ki,Kd)
        # Ki = 0
        # Kd = 0
        ts1, tp1, tr1, os1, ess1 = PID_Runner(Setpoint_Source, Systemstatus_Source, Kp, Ki, Kd)

        print("End of Source")
        print("Follow-up started")
        j = random.randint(1,4)
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
        elif(random_selection == 2):
            MR2_SP,MR2_SS,MR2_Kp,MR2_Ki,MR2_Kd = MR2(Kp)
            print("FOLLOW_UP VALUES MR2 : ",MR2_SP,MR2_SS,MR2_Kp,MR2_Ki,MR2_Kd)
            ts2,tp2,tr2,os2,ess2 = PID_Runner(MR2_SP,MR2_SS,MR2_Kp,MR2_Ki,MR2_Kd)

            if(tr1>=tr2 and os1<os2 and ess1>=ess2):
                print("no Fault detected by MR2")

            else:
                print("there is a Fault by MR2")
                if("MR2" in result):
                    result["MR2"] += 1
                else:
                    result["MR2"] = 1
                Faulty = True
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

if __name__ == '__main__':
    # MR1(10,12)
    num_trials = 10
    metamorphic_test(num_trials)
    # PID_Runner(8,7,2,0.01,0)
