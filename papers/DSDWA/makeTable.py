import numpy as np 
import sys
import matplotlib.pyplot as plt
from collections import OrderedDict
plt.rcParams["figure.figsize"] = [7.00, 7.00]
## Args: PythonAdrress Domain #Experiment #Policies #Weights DataAdrress

# weight_to_int = {'1.25':0, '1.50':1, '2.00':2, '3.00':3, '5.00':4, '9.00':5}
# weight_to_int = {'2.00':0, '3.00':1, '4.00':2, '5.00':3, '6.00':4, '7.00':5, '8.00':6, '9.00':7, '10.00':8}
# weight_to_int = {'2.0':0, '3.0':1, '4.0':2, '5.0':3, '6.0':4, '7.0':5, '8.0':6, '9.0':7, '10.0':8}
# weight_to_int = {"1.001":0, "1.002":1, "1.004":2, "1.008":3, "1.016":4, "1.032":5, "1.064":6, "1.128":7, "1.256":8, "1.512":9, "2.024":10, "3.048":11, "5.096":12}
# weight_to_int = {"1.01":0, "1.02":1, "1.04":2, "1.08":3, "1.16":4, "1.32":5, "1.64":6, "2.28":7, "3.56":8, "6.12":9, "11.24":10}
# weight_to_int = {"1.01":0, "1.02":1, "1.03":2, "1.04":3, "1.05":4, "1.06":5, "1.07":6, "1.08":7, "1.09":8, "1.10":9, "1.20":10, "1.30":11, "1.40":12, "1.50":13, "1.60":14, "1.70":15, "1.80":16, "1.90":17, "2.00":18, "2.10":19, "2.20":20, "2.30":21, "2.40":22, "2.50":23, "2.60":24, "2.70":25, "2.80":26, "2.90":27, "3.00":28, "3.10":29, "3.20":30, "3.30":31, "3.40":32, "3.50":33, "3.60":34, "3.70":35, "3.80":36, "3.90":37, "4.00":38, "4.10":39, "4.20":40, "4.30":41, "4.40":42, "4.50":43, "4.60":44, "4.70":45, "4.80":46, "4.90":47, "5.00":48}
# weight_to_int = {"1.10":0, "1.20":1, "1.30":2, "1.40":3, "1.50":4, "1.60":5, "1.70":6, "1.80":7, "1.90":8, "2.00":9, "2.10":10, "2.20":11, "2.30":12, "2.40":13, "2.50":14, "2.60":15, "2.70":16, "2.80":17, "2.90":18, "3.00":19, "3.10":20, "3.20":21, "3.30":22, "3.40":23, "3.50":24, "3.60":25, "3.70":26, "3.80":27, "3.90":28, "4.00":29, "4.10":30, "4.20":31, "4.30":32, "4.40":33, "4.50":34, "4.60":35, "4.70":36, "4.80":37, "4.90":38, "5.00":39}
# weight_to_int = {"1.12":0, "1.25":1, "1.50":2, "2.00":3, '3.00':4, '4.00':5, '5.00':6, '6.00':7, '7.00':8, '8.00':9, '9.00':10, '10.00':11}
weight_to_int = {"1.50":0, "2.00":1, '3.00':2, '4.00':3, '5.00':4, '6.00':5, '7.00':6, '8.00':7, '9.00':8, '10.00':9}

# int_to_alg = {0:'WA*', 1:'PWXD', 2:'PWXU', 3:'XDP', 4:'XUP', 5:'CADP'}
int_to_alg = {0:'WA*', 1:'PWXD', 2:'PWXU', 3:'XDP', 4:'XUP', 5:'OBDP'}

cost='5.0'
mapType = {0:'Obstacle Square', 1:'Swamped Square Cost='+cost, 2:'Obstacle Diamond', 3:'Swamped Diamond Cost='+cost, 4:'Obstacle Circle', 5:'Swamped Circle Cost='+cost, 6: sys.argv[5]+" Cost="+sys.argv[8]}
markers = [',', '^', 'X', '8', 's', 'o']
colours = ['tab:gray', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:blue']
linestyles = OrderedDict(
    [('dashed',              (0, (5, 5))),
     ('dashdotted',          (0, (3, 5, 1, 5))),
     ('dotted',              (0, (1, 5))),
     ('long dash with offset', (5, (10, 3))),
     ('dashdotdotted',         (0, (3, 5, 1, 5, 1, 5))),
     ('solid',               (0, ())),
     ('loosely dotted',      (0, (1, 10))),
     ('densely dashed',      (0, (5, 1))),
     ('densely dotted',      (0, (1, 1))),
     ('loosely dashed',      (0, (5, 10))),
     ('loosely dashdotted',  (0, (3, 10, 1, 10))),
     ('densely dashdotted',  (0, (3, 1, 1, 1))),
     ('loosely dashdotdotted', (0, (3, 10, 1, 10, 1, 10))),
     ('densely dashdotdotted', (0, (3, 1, 1, 1, 1, 1)))])

if sys.argv[1] == '-stp':
    if sys.argv[2] == '1':

        ##Experiment 1: Creates a table from average of runs of different problems over different weights
        table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        count_table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        with open("./papers/DSDWA/results/"+sys.argv[5]+".txt", "r") as f:
            for line in f:
                data = line.split()
                if(len(data)): ## To check for empy lines
                    if data[0] == "STP" and (data[5] in list(weight_to_int.keys())) and (int(data[3]) in list(int_to_alg.keys())) : #and data[3]!='0' and data[3]!='1' and data[3]!='6' and data[3]!='8': # and data[9]!='0':
                        table[int(data[3])][weight_to_int[data[5]]] += float(data[7])
                        count_table[int(data[3])][weight_to_int[data[5]]] += 1
                        # print(float(data[7]), count_table[int(data[3])][weight_to_int[data[5]]], sep=" ")

        result = np.divide(table, count_table)

        # divisor=np.array([2,3,4])
        # table/(divisor[:,np.newaxis])
        print()
        # print('============================================== Average Expansions Table ==============================================')
        # print('Algorithm/Weight|      1.25      |      1.50      |      2.00      |      3.00      |      5.00      |      9.00      |')
        print('====================================================  Average Expansions Table  =====================================================')
        print('Alg / Weight|   1.50    |   2.00    |   3.00    |   4.00    |   5.00    |   6.00    |   7.00    |   8.00    |   9.00    |   10.0    |')
        # print('====================================================  Average Expansions Table  =====================================================')
        # print('Alg / Weight|   6.00    |   7.00    |   8.00    |   9.00    |   10.00    |   11.00   |   12.00   |   13.00   |   14.00   |   15.0    |')
        print('_________________' * 7)
        for i in range(len(table)):
            # if i!=0 and i!=1 and i!=6 and i!=8:
            if i!=-1:
                print(int_to_alg[i],end="")
                for k in range(12-len(int_to_alg[i])):
                    print(' ',end="")
                print('|', end="")
                for j in range(len(table[i])):
                    print(round(result[i][j], 2), end="")
                    for k in range(11-len(str(round(result[i][j], 2)))):
                        print(' ',end="")
                    print('|', end="")
                print()
        print('__________________' * 7)
        for cnt in range(len(table)):
            print(str(cnt+1)+' place Alg |', end="")
            for i in range(len(table[0])):
                col = table[:,i]
                print(int_to_alg[np.argsort(col)[cnt]], end="")
                # print(int_to_alg[np.argmin(col)], end="")
                for k in range(11-len(int_to_alg[np.argsort(col)[cnt]])):
                    print(' ',end="")
                print('|', end="")
            print()
        
        print('=====================================================',end='')
        for _ in range((28 - len(sys.argv[5]))//2):
            print(' ', end='')
        print(sys.argv[5], end='')
        for _ in range((28 - len(sys.argv[5]))//2):
            print(' ', end='')
        print('====================================================')
        print()

        with open("./papers/DSDWA/results/"+sys.argv[5]+"_table.txt", "w") as f:
            for i in range(len(table)):
                if i!=6:
                    f.write("$\Phi_{\\text{"+int_to_alg[i]+"}}$ ")
                    for j in range(len(table[i])):
                        if  j!=3 and j!=5 and j!=6 and j!=7 and j!=9:
                            tmp = str(round(result[i][j], 2))
                            firstPart = tmp.split(".")[0]
                            secondPart = tmp.split(".")[1]
                            f.write("& ")
                            for k in range(len(firstPart)):
                                f.write(firstPart[k])
                                if (len(firstPart) - k)%4 == 0:
                                    f.write(",")
                            if len(secondPart):
                                f.write(".")
                            f.write(secondPart)
                            f.write(" ")
                            # f.write("& " + str(round(result[i][j], 2))+" ")
                    f.write("\\\\ \n \n")

    ##############################################
    elif sys.argv[2] == '2':
        ##Experiment 2: creates a plot for one specific weight sorting the hardness of problems
        # x-axis is all the problems in decending order and y-axix is the umber of node expansions

        dataset = [{} for _ in range(int(sys.argv[3]))]
        problems_hardness = {}
        for i in range(1,101):
            problems_hardness[i] = 0

        with open("./papers/DSDWA/results.txt", "r") as f:
            for line in f:
                data = line.split()
                if data[0] == 'STP' and data[5] == '1.20':# and data[9]!='0':
                    problems_hardness[int(data[1])] += int(data[7])
                    dataset[int(data[3])][int(data[1])] = int(data[7])

        sorted_hardness = sorted(problems_hardness.items(), key=lambda x:x[1], reverse=True)
        sorted_hardness_dict = {}
        for i in (sorted_hardness):
            sorted_hardness_dict[i[0]] = i[1]

        print("HARDEST PROBLEM IS: ", sorted_hardness_dict)

        x_ax = list(sorted_hardness_dict.keys())
        x_axis = [str(i) for i in x_ax]

        y_axis = [[] for _ in range(int(sys.argv[3]))]

        for i in range(int(sys.argv[3])):
            for p in x_axis:
                if int(p) in dataset[i]:
                    y_axis[i].append(dataset[i][int(p)])
                else:
                    y_axis[i].append(0)

        # plt.plot(x_axis, y_axis[0], 'ko-', label='WA*')
        # plt.plot(x_axis, y_axis[1], 'g*-', label='XDP')
        # plt.plot(x_axis, y_axis[2], 'bs-', label='XUP')
        # plt.plot(x_axis, y_axis[3], 'rv-', label='HalfEdgeDrop')
        # plt.legend(loc='upper right')
        # plt.show()

    ##############################################
    elif sys.argv[2] == '3':
        ##Experiment 3: For each problem, creates a plot of applying different algorithms using different weights

        for problem in range(81, 83):
            dataset = [{} for _ in range(int(sys.argv[3]))]

            with open("./papers/DSDWA/results/STP-results.txt", "r") as f:
                for line in f:
                    data = line.split()
                    if data[0] == 'STP' and data[1] ==str(problem):# and data[9]!='0':
                        dataset[int(data[3])][float(data[5])] = int(data[7]) #dataset[1(xdp)][1.20] = 23455

            x_axis = list(dataset[0].keys())
            y_axis = [list(i.values()) for i in dataset]

            # for i in range(int(sys.argv[3])):
                # plt.plot(x_axis, y_axis[i], markers[i], label=int_to_alg[i])
                

            # plt.legend(loc='upper right')
            # plt.yscale('log')
            # plt.xlabel('weight')
            # plt.ylabel('Node Expansions')
            # plt.title('STP Problem '+str(problem))
            # plt.show()

    ##############################################
    elif sys.argv[2] == '4':

        ##Experiment 4: Creates the work/weight plot
        ##arg[3] is the number of algs and arg[4] is the number of weights
        table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        count_table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        with open("./papers/DSDWA/results/"+sys.argv[5]+".txt", "r") as f:
            for line in f:
                data = line.split()
                if(len(data)): ## To check for empy lines
                    if data[0] == "STP" and (data[5] in list(weight_to_int.keys())) and (int(data[3]) in list(int_to_alg.keys())) : #and data[3]!='0' and data[3]!='1' and data[3]!='6' and data[3]!='8': # and data[9]!='0':
                        table[int(data[3])][weight_to_int[data[5]]] += float(data[7])
                        count_table[int(data[3])][weight_to_int[data[5]]] += 1
                        # print(float(data[7]), count_table[int(data[3])][weight_to_int[data[5]]], sep=" ")

        result = np.divide(table, count_table)
        # Weights = [2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
        Weights = [1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
        # Weights = [1.001, 1.002, 1.004, 1.008, 1.016, 1.032, 1.064, 1.128, 1.256, 1.512, 2.024, 3.048, 5.096]
        # Weights = [1.01, 1.02, 1.04, 1.08, 1.16, 1.32, 1.64, 2.28, 3.56, 6.12, 11.24]
        # Weights = [1.01, 1.02, 1.04, 1.08, 1.16, 1.32, 1.64, 2.28, 3.56, 6.12]
        # Weights = ["1.01", "1.02", "1.03", "1.04", "1.05", "1.06", "1.07", "1.08", "1.09", "1.10", "1.20", "1.30", "1.40", "1.50", "1.60", "1.70", "1.80", "1.90", "2.00", "2.10", "2.20", "2.30", "2.40", "2.50", "2.60", "2.70", "2.80", "2.90", "3.00", "3.10", "3.20", "3.30", "3.40", "3.50", "3.60", "3.70", "3.80", "3.90", "4.00", "4.10", "4.20", "4.30", "4.40", "4.50", "4.60", "4.70", "4.80", "4.90", "5.00"]
        # Weights = ["1.10", "1.20", "1.30", "1.40", "1.50", "1.60", "1.70", "1.80", "1.90", "2.00", "2.10", "2.20", "2.30", "2.40", "2.50", "2.60", "2.70", "2.80", "2.90", "3.00", "3.10", "3.20", "3.30", "3.40", "3.50", "3.60", "3.70", "3.80", "3.90", "4.00", "4.10", "4.20", "4.30", "4.40", "4.50", "4.60", "4.70", "4.80", "4.90", "5.00"]
        # Weights = ["11.00", "12.00", "13.00", "14.00", "15.00", "16.00", "17.00", "18.00", "19.00", "20.00", "21.00", "22.00", "23.00", "24.00", "25.00", "26.00", "27.00", "28.00", "29.00", "30.00", "31.00", "32.00", "33.00", "34.00", "35.00", "36.00", "37.00", "38.00", "39.00", "40.00", "41.00", "42.00", "43.00", "44.00", "45.00", "46.00", "47.00", "48.00", "49.00", "50.00"]
        # Weights = ["110.00", "120.00", "130.00", "140.00", "150.00", "160.00", "170.00", "180.00", "190.00", "200.00", "210.00", "220.00", "230.00", "240.00", "250.00", "260.00", "270.00", "280.00", "290.00", "300.00", "310.00", "320.00", "330.00", "340.00", "350.00", "360.00", "370.00", "380.00", "390.00", "400.00", "410.00", "420.00", "430.00", "440.00", "450.00", "460.00", "470.00", "480.00", "490.00", "500.00"]
        xpoints = np.array(Weights)
        works = []
        for policy in int_to_alg:
            work_i = []
            for w in list(weight_to_int.keys()):
                work_i.append(result[policy][weight_to_int[w]])
            works.append(work_i)
        for policy in int_to_alg:
            ypoints = []
            for i in works[policy]:
                ypoints.append(i)
            ypoints = np.array(ypoints)
            if policy!=5:
                plt.plot(xpoints, ypoints, linestyle=linestyles[list(linestyles.keys())[5]], linewidth = '2.5', marker=markers[policy], ms='10', markeredgecolor="k", label=int_to_alg[policy], color=colours[policy], alpha=0.5)
            else:
                plt.plot(xpoints, ypoints, linestyle=linestyles[list(linestyles.keys())[5]], linewidth = '2.5', marker=markers[policy], ms='10', markeredgecolor="k", label=int_to_alg[policy], color=colours[policy])
        
        font = {'family':'serif','color':'darkred','size':12}
        plt.ylabel("Work", fontdict=font)
        plt.xlabel("Weights", fontdict=font)
        plt.title(mapType[int(sys.argv[7])]+", Size="+str(sys.argv[6]))
        plt.legend(fontsize="17")
        plt.xticks(xpoints) 
        plt.yscale('log')
        plt.show()
############################################################################################
elif sys.argv[1] == '-map':
    if sys.argv[2] == '1':

        ##Experiment 1: Creates a table from average of runs of different problems over different weights
        table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        count_table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        with open("./papers/DSDWA/results/"+sys.argv[5]+".txt", "r") as f:
            for line in f:
                data = line.split()
                if(len(data)): ## To check for empy lines
                    if data[0] == "MAP" and (data[7] in list(weight_to_int.keys())) and (int(data[5]) in list(int_to_alg.keys())): #and data[5]!='0' and data[5]!='1' and data[5]!='6' and data[5]!='8':# and data[9]!='0':
                        table[int(data[5])][weight_to_int[data[7]]] += int(data[9])
                        count_table[int(data[5])][weight_to_int[data[7]]] += 1

        result = np.divide(table, count_table)
        print()
        # print('============================================== Average Expansions Table ==============================================')
        # print('Algorithm/Weight|      1.25      |      1.50      |      2.00      |      3.00      |      5.00      |      9.00      |')
        # print('==================================================  Average Expansions Table  ===================================================')
        # print('Alg / Weight|    2.00    |    3.00    |    4.00    |    5.00    |    6.00    |    7.00    |    8.00    |    9.00    |    10.0    |')
        # print('====================================================  Average Expansions Table  =====================================================')
        # print('Alg / Weight|  1.12   |  1.25   |  1.50   |  2.00   |  3.00   |  4.00   |  5.00   |  6.00   |  7.00   |  8.00   |  9.00   |  10.0   |')
        print('==========================================  Average Expansions Table  ===========================================')
        print('Alg / Weight|  1.50   |  2.00   |  3.00   |  4.00   |  5.00   |  6.00   |  7.00   |  8.00   |  9.00   |  10.0   |')
        print('_________________' * 7)
        for i in range(len(table)):
            # if i!=0 and i!=1 and i!=6 and i!=8:
            if i!=6:
                print(int_to_alg[i],end="")
                for k in range(12-len(int_to_alg[i])):
                    print(' ',end="")
                print('|', end="")
                for j in range(len(table[i])):
                    print(round(result[i][j], 2), end="")
                    for k in range(9-len(str(round(result[i][j], 2)))):
                        print(' ',end="")
                    print('|', end="")
                print()
        print('__________________' * 7)
                    
        for cnt in range(len(table)):
            print(str(cnt+1)+' place Alg |', end="")
            for i in range(len(table[0])):
                col = table[:,i]
                print(int_to_alg[np.argsort(col)[cnt]], end="")
                # print(int_to_alg[np.argmin(col)], end="")
                for k in range(9-len(int_to_alg[np.argsort(col)[cnt]])):
                    print(' ',end="")
                print('|', end="")
            print()
        
        print('=====================================================',end='')
        for _ in range((26 - len(sys.argv[5]))//2):
            print(' ', end='')
        print(sys.argv[5], end='')
        for _ in range((26 - len(sys.argv[5]))//2):
            print(' ', end='')
        print('====================================================')
        print()

        with open("./papers/DSDWA/results/"+sys.argv[5]+"_table.txt", "w") as f:
            for i in range(len(table)):
                if i!=6:
                    f.write("$\Phi_{\\text{"+int_to_alg[i]+"}}$ ")
                    for j in range(len(table[i])):
                        if  j!=3 and j!=5 and j!=6 and j!=7 and j!=9:
                            tmp = str(round(result[i][j], 2))
                            firstPart = tmp.split(".")[0]
                            secondPart = tmp.split(".")[1]
                            f.write("& ")
                            for k in range(len(firstPart)):
                                f.write(firstPart[k])
                                if (len(firstPart) - k)%4 == 0:
                                    f.write(",")
                            if len(secondPart):
                                f.write(".")
                            f.write(secondPart)
                            f.write(" ")
                            # f.write("& " + str(round(result[i][j], 2))+" ")
                    f.write("\\\\ \n \n")

    ##############################################
    elif sys.argv[2] == '2':
        # Second idea: create plots, each plot for one weight, 
        # x-axis is all the problems in decending order and y-axix is the umber of node expansions

        dataset = [{} for _ in range(int(sys.argv[3]))]
        problems_hardness = {}
        for i in range(1,101):
            problems_hardness[i] = 0

        with open("./papers/DSDWA/results.txt", "r") as f:
            for line in f:
                data = line.split()
                if data[0] == 'MAP' and data[5] == '1.20':# and data[9]!='0':
                    problems_hardness[int(data[1])] += int(data[7])
                    dataset[int(data[3])][int(data[1])] = int(data[7])

        sorted_hardness = sorted(problems_hardness.items(), key=lambda x:x[1], reverse=True)
        sorted_hardness_dict = {}
        for i in (sorted_hardness):
            sorted_hardness_dict[i[0]] = i[1]

        print("HARDEST PROBLEM IS: ", sorted_hardness_dict)

        x_ax = list(sorted_hardness_dict.keys())
        x_axis = [str(i) for i in x_ax]

        y_axis = [[] for _ in range(int(sys.argv[3]))]

        # print(dataset[0])
        for i in range(int(sys.argv[3])):
            for p in x_axis:
                if int(p) in dataset[i]:
                    # print(dataset[i][p])
                    y_axis[i].append(dataset[i][int(p)])
                else:
                    y_axis[i].append(0)

        # print(y_axis[0])
        # print(x_axis)
        # plt.plot(x_axis, y_axis[0], 'ko-', label='WA*')
        # plt.plot(x_axis, y_axis[1], 'g*-', label='XDP')
        # plt.plot(x_axis, y_axis[2], 'bs-', label='XUP')
        # plt.plot(x_axis, y_axis[3], 'rv-', label='HalfEdgeDrop')
        # plt.legend(loc='upper right')
        # plt.show()

    ##############################################
    elif sys.argv[2] == '3':
        ##Experiment 3: For each problem, creates a plot of applying different algorithms using different weights

        problemsList = []
        with open("./papers/DSDWA/ALL-random-40-results.txt", "r") as f:
            for line in f:
                data = line.split()
                if data[0] == 'MAP' and (data[1] not in problemsList):
                    problemsList.append(data[1])
        f.close()
        
        for problem in problemsList:
            numberOfScenarios = 0
            with open("./papers/DSDWA/ALL-random-40-results.txt", "r") as f:
                dataset = [{} for _ in range(int(sys.argv[3]))]

                for line in f:
                    data = line.split()
                    if data[0] == 'MAP' and data[1] == problem :# and data[11]!='0':
                        if float(data[7]) not in list(dataset[int(data[5])].keys()):
                            dataset[int(data[5])][float(data[7])] = 0

                        dataset[int(data[5])][float(data[7])] += int(data[9]) #dataset[1(xdp)][1.20] = 23455

                        numberOfScenarios += 1
                        

                x_axis = list(dataset[0].keys())
                y_axis = [list(alg.values()) for alg in dataset]

                # print(y_axis)
                # print(y_axis)
                numberOfScenarios /= int(sys.argv[3])
                numberOfScenarios /= int(sys.argv[4])
                print(numberOfScenarios)
                for alg in range(len(y_axis)):
                    for i in range(len(y_axis[alg])):
                        y_axis[alg][i] = y_axis[alg][i]/numberOfScenarios
                # print(y_axis)

                # for i in range(int(sys.argv[3])):
                    # plt.plot(x_axis, y_axis[i], markers[i], label=int_to_alg[i])
                    

                # plt.legend(loc='upper right')
                # plt.yscale('log')
                # plt.xlabel('weight')
                # plt.ylabel('Node Expansions')
                # plt.title('MAP Problem '+str(problem))
                # plt.show()
    
    ##############################################
    elif sys.argv[2] == '4':

        ##Experiment 4: Creates the work/weight plot
        ##arg[3] is the number of algs and arg[4] is the number of weights
        table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        count_table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        with open("./papers/DSDWA/results/"+sys.argv[5]+".txt", "r") as f:
            for line in f:
                data = line.split()
                if(len(data)): ## To check for empy lines
                    if data[0] == "MAP" : #and data[5]!='0' and data[5]!='1' and data[5]!='6' and data[5]!='8':# and data[9]!='0':
                        table[int(data[5])][weight_to_int[data[7]]] += int(data[9])
                        count_table[int(data[5])][weight_to_int[data[7]]] += 1

        result = np.divide(table, count_table)
        # Weights = [2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
        Weights = [1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
        # Weights = [1.001, 1.002, 1.004, 1.008, 1.016, 1.032, 1.064, 1.128, 1.256, 1.512, 2.024, 3.048, 5.096]
        # Weights = [1.01, 1.02, 1.04, 1.08, 1.16, 1.32, 1.64, 2.28, 3.56, 6.12, 11.24]
        # Weights = [1.01, 1.02, 1.04, 1.08, 1.16, 1.32, 1.64, 2.28, 3.56, 6.12]
        # Weights = ["1.01", "1.02", "1.03", "1.04", "1.05", "1.06", "1.07", "1.08", "1.09", "1.10", "1.20", "1.30", "1.40", "1.50", "1.60", "1.70", "1.80", "1.90", "2.00", "2.10", "2.20", "2.30", "2.40", "2.50", "2.60", "2.70", "2.80", "2.90", "3.00", "3.10", "3.20", "3.30", "3.40", "3.50", "3.60", "3.70", "3.80", "3.90", "4.00", "4.10", "4.20", "4.30", "4.40", "4.50", "4.60", "4.70", "4.80", "4.90", "5.00"]
        # Weights = ["1.10", "1.20", "1.30", "1.40", "1.50", "1.60", "1.70", "1.80", "1.90", "2.00", "2.10", "2.20", "2.30", "2.40", "2.50", "2.60", "2.70", "2.80", "2.90", "3.00", "3.10", "3.20", "3.30", "3.40", "3.50", "3.60", "3.70", "3.80", "3.90", "4.00", "4.10", "4.20", "4.30", "4.40", "4.50", "4.60", "4.70", "4.80", "4.90", "5.00"]
        # Weights = ["11.00", "12.00", "13.00", "14.00", "15.00", "16.00", "17.00", "18.00", "19.00", "20.00", "21.00", "22.00", "23.00", "24.00", "25.00", "26.00", "27.00", "28.00", "29.00", "30.00", "31.00", "32.00", "33.00", "34.00", "35.00", "36.00", "37.00", "38.00", "39.00", "40.00", "41.00", "42.00", "43.00", "44.00", "45.00", "46.00", "47.00", "48.00", "49.00", "50.00"]
        # Weights = ["110.00", "120.00", "130.00", "140.00", "150.00", "160.00", "170.00", "180.00", "190.00", "200.00", "210.00", "220.00", "230.00", "240.00", "250.00", "260.00", "270.00", "280.00", "290.00", "300.00", "310.00", "320.00", "330.00", "340.00", "350.00", "360.00", "370.00", "380.00", "390.00", "400.00", "410.00", "420.00", "430.00", "440.00", "450.00", "460.00", "470.00", "480.00", "490.00", "500.00"]
        xpoints = np.array(Weights)
        works = []
        for policy in int_to_alg:
            work_i = []
            for w in list(weight_to_int.keys()):
                work_i.append(result[policy][weight_to_int[w]])
            works.append(work_i)
        for policy in int_to_alg:
            ypoints = []
            for i in works[policy]:
                ypoints.append(i)
            ypoints = np.array(ypoints)
            if policy!=5:
                plt.plot(xpoints, ypoints, linestyle=linestyles[list(linestyles.keys())[5]], linewidth = '2.5', marker=markers[policy], ms='10', markeredgecolor="k", label=int_to_alg[policy], color=colours[policy], alpha=0.5)
            else:
                plt.plot(xpoints, ypoints, linestyle=linestyles[list(linestyles.keys())[5]], linewidth = '2.5', marker=markers[policy], ms='10', markeredgecolor="k", label=int_to_alg[policy], color=colours[policy])
        
        font = {'family':'serif','color':'darkred','size':12}
        plt.ylabel("Work", fontdict=font)
        plt.xlabel("Weights", fontdict=font)
        plt.title(mapType[int(sys.argv[7])]+", Size="+str(sys.argv[6]))
        plt.legend(fontsize="17")
        plt.xticks(xpoints) 
        plt.yscale('log')
        # plt.xscale('log')
        plt.show()
    
    ############################################################################################
    elif sys.argv[2] == '5':

        ##Experiment 5: Created the box plots. Each plot, y-axis:work and x-axis:algs, for one weight.
        ##arg[3] is the number of algs and arg[4] is the number of weights
        Weights = {"1.50":[], "2.00":[], '3.00':[], '4.00':[], '5.00':[], '6.00':[], '7.00':[], '8.00':[], '9.00':[], '10.00':[]}
        for w in Weights.values():
            for i in range(int(sys.argv[3])):
                a = []
                w.append(a)
        
        cnt = 0
        with open("./papers/DSDWA/results/"+sys.argv[5]+".txt", "r") as f:
            numLines = len(f.readlines())
        with open("./papers/DSDWA/results/"+sys.argv[5]+".txt", "r") as f:
            for line in f:
                data = line.split()
                if(len(data)): ## To check for empy lines
                    if data[0] == "MAP" : #and data[5]!='0' and data[5]!='1' and data[5]!='6' and data[5]!='8':# and data[9]!='0':
                        np.append(Weights[data[7]][int(data[5])],(int(data[9])))
                        Weights[data[7]][int(data[5])].append(int(data[9]))
                cnt += 1
                if cnt/numLines*100 % 5 == 0:
                    print(cnt/numLines*100, '%')

        print("Done Reading the Data..")
        for w in range(len(Weights.values())):
            data = []
            for alg in list(Weights.values())[w]:
                data.append(np.array(alg))
            
            print('Plotted w=', str(w))
            print([item.get_ydata()[1] for item in data])

            fig, ax = plt.subplots()
            ax.set_title(sys.argv[5] + 'Plot for Weight=' + str(w))
            ax.boxplot(data, showfliers=False, notch=True)
            y_pos = np.arange(len(int_to_alg.values())+1)
            labels = ['']
            labels += list(int_to_alg.values())


            plt.xticks(y_pos, labels)

            plt.show()