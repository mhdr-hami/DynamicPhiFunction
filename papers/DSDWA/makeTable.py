import numpy as np 
import sys
import matplotlib.pyplot as plt
## first arg is number of policies and second one is number of weights in the experiment.

# table = np.zeros((int(sys.argv[1]), int(sys.argv[2])))
# count_table = np.zeros((int(sys.argv[1]), int(sys.argv[2])))
# weight_to_int = {'1.20':0, '1.50':1, '2.00':2, '5.00':3, '10.00':4}
int_to_alg = {0:'WA*', 1:'XDP', 2:'XUP', 3:'HalfEdgeDrop', 4:'kGreedy', 5:'kFullEdgeDrop', 6:'kPathSuboptDouble', 7:'kMixed', 8:'kLastDelta'}
markers = ['o-', '*-', 's-', 'v-', '1-', 'p-', '+-', '-.', 'D-']
# with open("./papers/DSDWA/results.txt", "r") as f:
#     for line in f:
#         data = line.split()
#         if data[0] == "STP" :# and data[9]!='0':
#             table[int(data[3])][weight_to_int[data[5]]] += int(data[7])
#             count_table[int(data[3])][weight_to_int[data[5]]] += 1

# result = np.divide(table, count_table)
# print()
# print('====================================== Average Expansions Table ======================================')
# print('Algorithm/Weight|      1.25      |      1.50      |      2.00      |      3.00      |      5.00     |')
# for i in range(len(table)):
#     print(int_to_alg[i],end="")
#     for k in range(16-len(int_to_alg[i])):
#         print(' ',end="")
#     print('|', end="")
#     for j in range(len(table[i])):
#         print(result[i][j], end="")
#         for k in range(16-len(str(result[i][j]))):
#             print(' ',end="")
#         print('|', end="")
#     print()
# print()
# print()

# One question about this idea: what to do with huge runs? 
# if we should not ignore them, leave the comment here 
# and also chage the code to not to break after 20 million
############################################################################################

# Second idea: create plots, each plot for one weight, 
# x-axis is all the problems in decending order and y-axix is the umber of node expansions

# dataset = [{} for _ in range(int(sys.argv[1]))]
# problems_hardness = {}
# for i in range(1,101):
#     problems_hardness[i] = 0

# with open("./papers/DSDWA/results.txt", "r") as f:
#     for line in f:
#         data = line.split()
#         if data[0] == 'STP' and data[5] == '1.20':# and data[9]!='0':
#             problems_hardness[int(data[1])] += int(data[7])
#             dataset[int(data[3])][int(data[1])] = int(data[7])

# sorted_hardness = sorted(problems_hardness.items(), key=lambda x:x[1], reverse=True)
# sorted_hardness_dict = {}
# for i in (sorted_hardness):
#     sorted_hardness_dict[i[0]] = i[1]

# print("HARDEST PROBLEM IS: ", sorted_hardness_dict)

# x_ax = list(sorted_hardness_dict.keys())
# x_axis = [str(i) for i in x_ax]

# y_axis = [[] for _ in range(int(sys.argv[1]))]

# # print(dataset[0])
# for i in range(int(sys.argv[1])):
#     for p in x_axis:
#         if int(p) in dataset[i]:
#             # print(dataset[i][p])
#             y_axis[i].append(dataset[i][int(p)])
#         else:
#             y_axis[i].append(0)

# # print(y_axis[0])
# # print(x_axis)
# plt.plot(x_axis, y_axis[0], 'ko-', label='WA*')
# plt.plot(x_axis, y_axis[1], 'g*-', label='XDP')
# plt.plot(x_axis, y_axis[2], 'bs-', label='XUP')
# plt.plot(x_axis, y_axis[3], 'rv-', label='HalfEdgeDrop')
# plt.legend(loc='upper right')
# plt.show()


############################################################################################

# Third idea: create plots like the ones in the paper based on the bound and number of expansions
## For one problem and different weights, compares te algorithms
for problem in range(80, 82):
    dataset = [{} for _ in range(int(sys.argv[1]))]

    with open("./papers/DSDWA/results.txt", "r") as f:
        for line in f:
            data = line.split()
            if data[0] == 'STP' and data[1] ==str(problem):# and data[9]!='0':
                dataset[int(data[3])][float(data[5])] = int(data[7]) #dataset[1(xdp)][1.20] = 23455

    x_axis = list(dataset[0].keys())
    y_axis = [list(i.values()) for i in dataset]

    for i in range(int(sys.argv[1])):
        plt.plot(x_axis, y_axis[i], markers[i], label=int_to_alg[i])

    plt.legend(loc='upper right')
    plt.xlabel('weight')
    plt.ylabel('Node Expansions')
    plt.title('Problem '+str(problem))
    # plt(num='Problem '+str(problem))
    plt.show()