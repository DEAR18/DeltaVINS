import matplotlib.pyplot as plt
import associate
import sys
import numpy
import evaluate_ate

def plotTrajectory(fileName,gt):

    data_list = associate.read_file_list(fileName)
    gt_list = associate.read_file_list(gt)
    matches = associate.associate(data_list,gt_list,0,0.02)
    if len(matches) < 2:
        sys.exit(
            "Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")
    matches.pop()

    first_xyz = numpy.matrix([[float(value) for value in data_list[a][0:3]] for a,b in matches]).transpose()
    second_xyz = numpy.matrix([[float(value) for value in gt_list[b][0:3]] for a,b in matches]).transpose()
    rot,trans,trans_error = evaluate_ate.align(first_xyz,second_xyz)

    timestamp =[]
    gtimestamp =[]
    for value in data_list.keys():
        timestamp.append(value)
    for value in gt_list.keys():
        gtimestamp.append(value)

    values = list(data_list.values())
    gvalues = list(gt_list.values())

    data_xyz_full = numpy.matrix([[float(value) for value in data_list[b][0:3]] for b in timestamp]).transpose()
    gt_xyz_full = numpy.matrix([[float(value) for value in gt_list[b][0:3]] for b in gtimestamp]).transpose()
    gt_xyz_full_aligned = rot * gt_xyz_full + trans

    x=[]
    y=[]
    z=[]
    gz=[]
    gy=[]
    gx=[]

    for data in values:
        x.append(float(data[0]))
        y.append(float(data[1]))
        z.append(float(data[2]))


    for gtdata in gvalues:
        gx.append(float(gtdata[0]))
        gy.append(float(gtdata[1]))
        gz.append(float(gtdata[2]))


    plt.plot(timestamp,y,label='y')
    plt.plot(gtimestamp,gx,label='gy')
    plt.title('y axis')

    plt.figure()
    plt.plot(x,z,color='r')
    plt.plot([x[0]],[z[0]],marker='o',color='r',label='Start Point')
    plt.plot([x[-1]],[z[-1]],marker='o',color='g',label='End Point')


    plt.plot(gy,gz,color='g')
    plt.plot([gy[0]],[gz[0]],marker='o',label='Start Point')
    plt.plot([gy[-1]],[gz[-1]],marker='x',label='End Point')

    plt.title('x-z axis')
    plt.legend()


    plt.show()














