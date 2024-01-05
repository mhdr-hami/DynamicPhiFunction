import numpy as np 
import sys
import matplotlib.pyplot as plt
plt.rcParams["figure.figsize"] = [9.00, 7.00]
## Args: PythonAdrress Domain #Experiment #Policies #Weights DataAdrress

weight_to_int = {'1.25':0, '1.50':1, '2.00':2, '3.00':3, '5.00':4, '9.00':5}
int_to_alg = {0:'kNineth', 1:'kSixth', 2:'kPathSuboptDoub', 3:'kpwXUP', 4:'kpwXDP', 5:'WA*', 6:'kTenth', 7:'kSuper', 8:'kX', 9:'kXDP90', 10:'kTheOne'}
markers = ['o-', '*-', 's-', 'v-', '1-', 'p-', '+-', '-.', '-.', '-.', 'D-']

if sys.argv[1] == '-stp':
    if sys.argv[2] == '1':

        ##Experiment 1: Creates a table from average of runs of different problems over different weights
        table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        count_table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        with open("./papers/DSDWA/results/"+sys.argv[5]+"-results.txt", "r") as f:
            for line in f:
                data = line.split()
                if data[0] == "STP" and data[3]!='0' and data[3]!='1' and data[3]!='6' and data[3]!='8': # and data[9]!='0':
                    table[int(data[3])][weight_to_int[data[5]]] += int(data[7])
                    count_table[int(data[3])][weight_to_int[data[5]]] += 1

        result = np.divide(table, count_table)
        print()
        print('============================================== Average Expansions Table ==============================================')
        print('Algorithm/Weight|      1.25      |      1.50      |      2.00      |      3.00      |      5.00      |      9.00      |')
        print('_________________' * 7)
        for i in range(len(table)):
            if i!=0 and i!=1 and i!=6 and i!=8:
                print(int_to_alg[i],end="")
                for k in range(16-len(int_to_alg[i])):
                    print(' ',end="")
                print('|', end="")
                for j in range(len(table[i])):
                    print(round(result[i][j], 2), end="")
                    for k in range(16-len(str(round(result[i][j], 2)))):
                        print(' ',end="")
                    print('|', end="")
                print()
        print('_________________' * 7)

        for cnt in range(4, 11):
            print(str(cnt-3)+' Best Algorithm|', end="")
            for i in range(len(table[0])):
                col = table[:,i]
                print(int_to_alg[np.argsort(col)[cnt]], end="")
                
                for k in range(16-len(int_to_alg[np.argsort(col)[cnt]])):
                    print(' ',end="")
                print('|', end="")
            print()

        print('==============================================',end='')
        for _ in range((26 - len(sys.argv[5]))//2):
            print(' ', end='')
        print(sys.argv[5], end='')
        for _ in range((26 - len(sys.argv[5]))//2):
            print(' ', end='')
        print('==============================================')
        print()

    ############################################################################################
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

        plt.plot(x_axis, y_axis[0], 'ko-', label='WA*')
        plt.plot(x_axis, y_axis[1], 'g*-', label='XDP')
        plt.plot(x_axis, y_axis[2], 'bs-', label='XUP')
        plt.plot(x_axis, y_axis[3], 'rv-', label='HalfEdgeDrop')
        plt.legend(loc='upper right')
        plt.show()

    ############################################################################################
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

            for i in range(int(sys.argv[3])):
                plt.plot(x_axis, y_axis[i], markers[i], label=int_to_alg[i])
                

            plt.legend(loc='upper right')
            plt.yscale('log')
            plt.xlabel('weight')
            plt.ylabel('Node Expansions')
            plt.title('STP Problem '+str(problem))
            plt.show()

elif sys.argv[1] == '-map':
    if sys.argv[2] == '1':

        ##Experiment 1: Creates a table from average of runs of different problems over different weights
        table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        count_table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        with open("./papers/DSDWA/results/"+sys.argv[5]+"-results.txt", "r") as f:
            for line in f:
                data = line.split()
                if data[0] == "MAP" and data[5]!='0' and data[5]!='1' and data[5]!='6' and data[5]!='8':# and data[9]!='0':
                    table[int(data[5])][weight_to_int[data[7]]] += int(data[9])
                    count_table[int(data[5])][weight_to_int[data[7]]] += 1

        result = np.divide(table, count_table)
        print()
        print('============================================== Average Expansions Table ==============================================')
        print('Algorithm/Weight|      1.25      |      1.50      |      2.00      |      3.00      |      5.00      |      9.00      |')
        print('_________________' * 7)
        for i in range(len(table)):
            if i!=0 and i!=1 and i!=6 and i!=8:
                print(int_to_alg[i],end="")
                for k in range(16-len(int_to_alg[i])):
                    print(' ',end="")
                print('|', end="")
                for j in range(len(table[i])):
                    print(round(result[i][j], 2), end="")
                    for k in range(16-len(str(round(result[i][j], 2)))):
                        print(' ',end="")
                    print('|', end="")
                print()
        print('_________________' * 7)
        for cnt in range(4, 11):
            print(str(cnt-3)+' Best Algorithm|', end="")
            for i in range(len(table[0])):
                col = table[:,i]
                print(int_to_alg[np.argsort(col)[cnt]], end="")
                # print(int_to_alg[np.argmin(col)], end="")
                for k in range(16-len(int_to_alg[np.argsort(col)[cnt]])):
                    print(' ',end="")
                print('|', end="")
            print()
        
        print('==============================================',end='')
        for _ in range((26 - len(sys.argv[5]))//2):
            print(' ', end='')
        print(sys.argv[5], end='')
        for _ in range((26 - len(sys.argv[5]))//2):
            print(' ', end='')
        print('==============================================')
        print()

    ############################################################################################
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
        plt.plot(x_axis, y_axis[0], 'ko-', label='WA*')
        plt.plot(x_axis, y_axis[1], 'g*-', label='XDP')
        plt.plot(x_axis, y_axis[2], 'bs-', label='XUP')
        plt.plot(x_axis, y_axis[3], 'rv-', label='HalfEdgeDrop')
        plt.legend(loc='upper right')
        plt.show()

    ############################################################################################
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

                for i in range(int(sys.argv[3])):
                    plt.plot(x_axis, y_axis[i], markers[i], label=int_to_alg[i])
                    

                plt.legend(loc='upper right')
                plt.yscale('log')
                plt.xlabel('weight')
                plt.ylabel('Node Expansions')
                plt.title('MAP Problem '+str(problem))
                plt.show()
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

                for i in range(int(sys.argv[3])):
                    plt.plot(x_axis, y_axis[i], markers[i], label=int_to_alg[i])
                    

                plt.legend(loc='upper right')
                plt.yscale('log')
                plt.xlabel('weight')
                plt.ylabel('Node Expansions')
                plt.title('MAP Problem '+str(problem))
                plt.show()