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
        P = Kp * (beta * SP - PV)
        # print(t,P,SP,PV)
        I = I + Ki * (SP - PV) * (t - t_prev)
        eD = gamma * SP - PV
        D = Kd * (eD - eD_prev) / (t - t_prev)
        MV = MV_bar + P + I + D

        # update stored data for next iteration
        eD_prev = eD
        t_prev = t


def PID_Runner(Setpoint, System_status, kp, ki, kd):
    TCLab = setup(connected=False, speedup=10)

    controller = PID(kp, ki, kd, MV_bar=System_status)  # create pid control
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

        setpoint = Setpoint
        calculation = {}

        for t in clock(tfinal, 2):
            # SP = T1 if t < 50 or (t > 500 and t < 750) else 50           # get setpoint
            SP = T1 if t < 50 else setpoint
            # SP = setpoint if t < 50 else setpoint
            PV = lab.T1  # get measurement

            MV = controller.send([t, PV, SP])  # compute manipulated variable

            calculation[int(t)] = PV

            lab.U1 = MV  # apply
            p.update(t)  # update information display

        peak_value = max(calculation.values())
        peak_time = list(calculation.keys())[list(calculation.values()).index(max(calculation.values()))]
        key_list = [key for key in calculation]
        common_values = {}
        for key in range(round(len(calculation) * 3 / 4), len(calculation)):
            if calculation[key_list[key]] in common_values:
                common_values[calculation[key_list[key]]] += 1
            else:
                common_values[calculation[key_list[key]]] = 1

        steady_state_value = list(common_values.keys())[list(common_values.values()).index(max(common_values.values()))]
        steady_state_error = abs(setpoint - steady_state_value)

        percentage_overshoot = calculation[peak_time] - steady_state_value

        before_rise = calculation[key_list[24]]
        for key in range(25, len(calculation)):
            if calculation[key_list[key]] > System_status and calculation[key_list[key]] != before_rise:
                growup_time = key_list[key]
                break

        rise_diff_start = 1
        rise_diff_stop = 1

        for key in range(len(calculation)):
            if growup_time < key_list[key] < peak_time:
                if abs((calculation[key_list[key]]) - (
                        abs(0.1 * (steady_state_value - System_status) + System_status))) < rise_diff_start:
                    rise_start_time = key_list[key]
                    rise_diff_start = abs(
                        (calculation[key_list[key]]) - (0.1 * (steady_state_value - System_status) + System_status))
                    rise_start_value = calculation[key_list[key]]

                if abs((calculation[key_list[key]]) - (
                        abs(0.9 * (steady_state_value - System_status) + System_status))) < rise_diff_stop:
                    rise_stop_time = key_list[key]
                    rise_diff_stop = abs(
                        (calculation[key_list[key]]) - (0.9 * (steady_state_value - System_status) + System_status))
                    rise_stop_value = calculation[key_list[key]]

        flag = False
        for key in range(len(calculation)):
            # if key_list[key] > peak_time:
            if (1.02 * steady_state_value) > (calculation[key_list[key]]) > (0.98 * steady_state_value) and not flag:
                settle_stop = key_list[key]
                # print(settle_stop)
                flag = True
            elif (1.02 * steady_state_value) < (calculation[key_list[key]]) or (calculation[key_list[key]]) < (
                    0.98 * steady_state_value):
                flag = False

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


def MR1(Setpoint_Source, System_status_Source, region, overall_region_number):
    Source_difference = Setpoint_Source - System_status_Source
    follow_difference = -1
    Setpoint_follow = 0
    Systemstatus_follow = 0

    down, up = range_detection(2, 10, region, overall_region_number)

    while follow_difference <= Source_difference:
        Setpoint_follow = random.uniform(down, up)
        Systemstatus_follow = random.uniform(down, up)
        follow_difference = Setpoint_follow - Systemstatus_follow

    Kp = random.uniform(down, up)
    down, up = range_detection(0.002, 0.01, region, overall_region_number)
    Ki = random.uniform(down, up)
    down, up = range_detection(0.5, 10, region, overall_region_number)
    Kd = random.uniform(down, up)

    return Setpoint_follow, Systemstatus_follow, Kp, Ki, Kd


def MR2(Kp, region, overall_region_number):
    down, up = range_detection(2, 10, region, overall_region_number)
    Setpoint_follow = random.uniform(down, up)
    Systemstatus_follow = random.uniform(down, up)
    while Setpoint_follow <= Systemstatus_follow:
        Setpoint_follow = random.uniform(down, up)
        Systemstatus_follow = random.uniform(down, up)
    Kp_follow = random.uniform(down, up)
    while Kp_follow <= Kp:
        Kp_follow = random.uniform(down, up)

    Ki = 0
    Kd = 0

    return Setpoint_follow, Systemstatus_follow, Kp_follow, Ki, Kd


def MR3(Ki, region, overall_region_number):
    down, up = range_detection(2, 10, region, overall_region_number)
    Setpoint_follow = random.uniform(down, up)
    Systemstatus_follow = random.uniform(down, up)
    while Setpoint_follow <= Systemstatus_follow:
        Setpoint_follow = random.uniform(down, up)
        Systemstatus_follow = random.uniform(down, up)
    # MR_diff = Setpoint_follow - Systemstatus_follow
    Kp = random.uniform(down, up)
    Ki_follow = -1
    down, up = range_detection(0.002, 0.01, region, overall_region_number)
    while Ki > Ki_follow:
        Ki_follow = random.uniform(down, up)

    down, up = range_detection(0.5, 10, region, overall_region_number)
    Kd = random.uniform(down, up)

    return Setpoint_follow, Systemstatus_follow, Kp, Ki_follow, Kd


def MR4(Kd, region, overall_region_number):
    down, up = range_detection(2, 10, region, overall_region_number)
    Setpoint_follow = random.uniform(down, up)
    # Setpoint_follow = 30
    Systemstatus_follow = random.uniform(down, up)
    # Systemstatus_follow = 0
    while (Setpoint_follow <= Systemstatus_follow):
        Setpoint_follow = random.uniform(down, up)
        Systemstatus_follow = random.uniform(down, up)

    MR_diff = Setpoint_follow - Systemstatus_follow
    Kp = random.uniform(down, up)
    down, up = range_detection(0.002, 0.01, region, overall_region_number)
    Ki = random.uniform(down, up)
    Kd_follow = -1
    down, up = range_detection(0.5, 10, region, overall_region_number)
    while Kd > Kd_follow:
        Kd_follow = random.uniform(down, up)

    return Setpoint_follow, Systemstatus_follow, Kp, Ki, Kd_follow


import matplotlib.pyplot as plt
import numpy as np


class Bandit:
    def __init__(self, p):
        # self.mean = mean
        # self.variance = 2
        self.p = p
        self.p_estimate = 0
        self.n = 0

    def pull(self, result):
        # return np.random.random() < self.p
        return result

    def update(self, x):
        self.n += 1
        self.p_estimate = ((self.n - 1) * self.p_estimate + x) / self.n


class BanditPartition:
    def __init__(self, p):
        # self.mean = mean
        # self.variance = 2
        self.p = p
        self.p_estimate = 0
        self.n = 0

    def pull(self, result):
        # return np.random.random() < self.p
        return result

    def update(self, x):
        self.n += 1
        self.p_estimate = ((self.n - 1) * self.p_estimate + x) / self.n


def check_for_eps(step, eps):
    if (step > 0 and step % 2 == 0):
        return eps - 0.01
    else:
        return eps


# def experiment(num_trials, eps, bandit_probs):
#     bandits = [Bandit(p) for p in bandit_probs]
#     rewards = np.zeros(num_trials)
#     num_times_explored = 0
#     num_times_exploited = 0
#     num_optimal = 0
#     optimal_j = np.argmax([bandit.p for bandit in bandits])
#     print("optimal j is : ", optimal_j)
#     for item in range(num_trials):
#         if (np.random.random() < eps):
#             num_times_explored += 1
#             j = np.random.randint(len(bandits))
#         else:
#             num_times_exploited += 1
#             j = np.argmax([b.p_estimate for b in bandits])
#         if (j == optimal_j):
#             num_optimal += 1
#         x = bandits[j].pull()
#         rewards[item] = x
#         bandits[j].update(x)
#         eps = check_for_eps(item, eps)
#
#     for b in bandits:
#         print("mean estimate (bandit-p_estimate) : ", b.p_estimate)
#     print("total reward earned", rewards.sum())
#     print("Overal win rate : ", rewards.sum() / num_trials)
#     print("num times explored : ", num_times_explored)
#     print("num times exploited : ", num_times_exploited)
#     print("num times optimal bandit selected : ", num_optimal)
#     cumulative_reward = np.cumsum(rewards)
#     win_rate = cumulative_reward / (np.arange(num_trials) + 1)
#     plt.plot(win_rate)
#     plt.plot(np.ones(num_trials) * np.max(bandit_probs))
#     plt.show()

def range_detection(down, up, region, number_of_regions):
    top = ((up - down) / number_of_regions * region) + down
    bottom = ((up - down) / number_of_regions * (region - 1)) + down
    return bottom, top


def metamorphic_test(num_trials, eps, bandit_probs, bandit_prob_partition):
    bandits = [Bandit(p) for p in bandit_probs]
    rewards = np.zeros(num_trials)
    num_times_explored = 0
    num_times_exploited = 0
    bandits_partition = [BanditPartition(p) for p in bandit_prob_partition]
    rewards_partition = np.zeros(num_trials)
    num_times_explored_partition = 0
    num_times_exploited_partition = 0
    # num_optimal = 0
    # optimal_j = np.argmax([bandit.p for bandit in bandits])
    total_region_number = 4
    result = {}
    result_partition = {}
    agents = {}
    result_partition = {}
    agents_partition = {}
    for i in range(0, num_trials):
        Faulty = False
        # region = np.random.randint(1,total_region_number+1)
        # ----------------------------------------
        ran1 = np.random.random()

        if ran1 < eps:
            num_times_explored_partition += 1
            k = np.random.randint(len(bandits_partition))
        else:
            num_times_exploited_partition += 1
            k = np.argmax([b.p_estimate for b in bandits_partition])

        k += 1
        if k in agents_partition:
            agents_partition[k] += 1
        else:
            agents_partition[k] = 1
        region = k
        ############################################
        down, up = range_detection(1, 8, region, total_region_number)
        Setpoint_Source = random.uniform(down, up)
        Systemstatus_Source = random.uniform(down, up)
        while Setpoint_Source <= Systemstatus_Source:
            Setpoint_Source = random.uniform(down, up)
            Systemstatus_Source = random.uniform(down, up)
        Source_difeerence = Setpoint_Source - Systemstatus_Source
        Kp = random.uniform(down, up)
        down, up = range_detection(0.001, 0.006, region, total_region_number)
        Ki = random.uniform(down, up)
        down, up = range_detection(0, 8, region, total_region_number)
        Kd = random.uniform(down, up)

        print(Setpoint_Source,Systemstatus_Source,Kp,Ki,Kd)
        ts1, tp1, tr1, os1, ess1 = PID_Runner(Setpoint_Source, Systemstatus_Source, Kp, Ki, Kd)

        print("End of Source")
        print("in Source we have : ", Setpoint_Source, Systemstatus_Source, Kp, Ki, Kd)
        print("Follow-up started")

        #######################################
        ran1 = np.random.random()
        if ran1 < eps:
            num_times_explored += 1
            j = np.random.randint(len(bandits))
        else:
            num_times_exploited += 1
            j = np.argmax([b.p_estimate for b in bandits])
        j += 1
        if j in agents:
            agents[j] += 1

        else:
            agents[j] = 1

        random_selection = j

        print("k,j is:", k, j)
        # ---------------------------------------------------

        if (random_selection == 1):
            # MR1_SP,MR1_SS,MR1_Kp,MR1_Ki,MR1_Kd = MR1(Setpoint_Source,Systemstatus_Source)
            MR1_SP, MR1_SS, MR1_Kp, MR1_Ki, MR1_Kd = MR1(Setpoint_Source, Systemstatus_Source, region,
                                                         total_region_number)

            print("FOLLOW_UP VALUES MR1 : ", MR1_SP, MR1_SS, MR1_Kp, MR1_Ki, MR1_Kd)

            ts2, tp2, tr2, os2, ess2 = PID_Runner(MR1_SP, MR1_SS, MR1_Kp, MR1_Ki, MR1_Kd)
            if (ts1 < ts2):
                print("no Fault detected by MR1")

            else:
                print("there is a Fault by MR1")
                if "MR1" in result:
                    result["MR1"] += 1
                else:
                    result["MR1"] = 1
                Faulty = True
        elif random_selection == 2:
            MR2_SP, MR2_SS, MR2_Kp, MR2_Ki, MR2_Kd = MR2(Kp, region, total_region_number)
            print("FOLLOW_UP VALUES MR2 : ", MR2_SP, MR2_SS, MR2_Kp, MR2_Ki, MR2_Kd)
            ts2, tp2, tr2, os2, ess2 = PID_Runner(MR2_SP, MR2_SS, MR2_Kp, MR2_Ki, MR2_Kd)

            if tr1 >= tr2 and os1 < os2 and ess1 >= ess2:
                print("no Fault detected by MR2")

            else:
                print("there is a Fault by MR2")
                if "MR2" in result:
                    result["MR2"] += 1
                else:
                    result["MR2"] = 1
                Faulty = True
        elif random_selection == 3:
            MR3_SP, MR3_SS, MR3_Kp, MR3_Ki, MR3_Kd = MR3(Ki, region, total_region_number)
            print("FOLLOW_UP VALUES MR3 : ", MR3_SP, MR3_SS, MR3_Kp, MR3_Ki, MR3_Kd)
            ts2, tp2, tr2, os2, ess2 = PID_Runner(MR3_SP, MR3_SS, MR3_Kp, MR3_Ki, MR3_Kd)
            if os1 < os2 and ess1 >= ess2 and ts1 <= ts2:
                print("no Fault detected by MR3")

            else:
                print("there is a Fault by MR3")
                if "MR3" in result:
                    result["MR3"] += 1
                else:
                    result["MR3"] = 1
                Faulty = True
        else:
            MR4_SP, MR4_SS, MR4_Kp, MR4_Ki, MR4_Kd = MR4(Kd, region, total_region_number)
            print("FOLLOW_UP VALUES MR4 : ", MR4_SP, MR4_SS, MR4_Kp, MR4_Ki, MR4_Kd)
            ts2, tp2, tr2, os2, ess2 = PID_Runner(MR4_SP, MR4_SS, MR4_Kp, MR4_Ki, MR4_Kd)
            if os1 >= os2 and ess1 >= ess2 and ts1 >= ts2:
                print("no Fault detected by MR4")

            else:
                print("there is a Fault by MR4")
                if "MR4" in result:
                    result["MR4"] += 1
                else:
                    result["MR4"] = 1
                Faulty = True

        if Faulty == True:
            if f'partition{k}' in result_partition:
                result_partition[f'partition{k}'] += 1
            else:
                result_partition[f'partition{k}'] = 1

        if Faulty == True:
            x = bandits[j - 1].pull(1)
            y = bandits_partition[k - 1].pull(1)
        else:
            x = bandits[j - 1].pull(0)
            y = bandits_partition[k - 1].pull(0)

        print("modified k,j is:", k, j)
        rewards[i] = x
        rewards_partition[i] = y
        print('rew is:', rewards[i])
        bandits[j-1].update(x)
        bandits_partition[k-1].update(y)
        eps = check_for_eps(i, eps)

        print(
            f"------------------------------------  Iteration {i + 1} was done -----------------------------------------------------")
    for item in range(len(bandits)):
        print(f'agnet{item+1} probability estimation: ', bandits[item].p_estimate)
        print(f'partition{item+1} probability estimation: ', bandits_partition[item].p_estimate)

    print('\n', '-------------------------------------------------------------')
    print('number of agent exploration is: ', num_times_explored, 'number of agent exploitation is: ',
          num_times_exploited)
    print('number of partition exploration is: ', num_times_explored_partition, 'number of partition exploitation is: ',
          num_times_exploited_partition)
    print('\n', '-------------------------------------------------------------')
    # print(rewards)
    print('number of times that each agent has selected')
    print(agents)
    print('number of times that each partition has selected')
    print(agents_partition)
    print('\n', '-------------------------------------------------------------')
    print('number of times that each agent appeared in Faulty execution')
    print(result)
    print('number of times that each partition appeared in Faulty execution')
    print(result_partition)


def main():
    num_trials = 10
    eps = 0.40
    bandit_prob = [0.25, 0.25, 0.25, 0.25]
    bandit_prob_partition = [0.25, 0.25, 0.25, 0.25]
    metamorphic_test(num_trials, eps, bandit_prob, bandit_prob_partition)


if __name__ == '__main__':
    main()
