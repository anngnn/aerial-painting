
import matplotlib.pyplot as plt
import csv
from mpl_toolkits.mplot3d import Axes3D
import sys
import numpy as np

NUMBER_OF_HEADER_ROWS = 2
X_COLUMN = 7
Y_COLUMN = 8
Z_COLUMN = 9
TIME_COLUMN = 2
X_SET_COLUMN = 19
Y_SET_COLUMN = 20
Z_SET_COLUMN = 21
X_KF_COLUMN = 32
Y_KF_COLUMN = 33
Z_KF_COLUMN = 34
FREQ_COLUMN = 4
MISSED_MSG_COLUMN = 29
X_INTEGRAL = 26
Y_INTEGRAL = 27
Z_INTEGRAL = 28
U_ROLL_COLUMN = 22
U_PITCH_COLUMN = 23
U_Z_COLUMN = 24
U_YAW_COLUMN = 25
ERROR_COLUMN = 31
FC_COLUMN = 32


def primary():
    try:
        print("now running", sys.argv[0])

        #First, get the name of the file (should be the second thing in the arguments)
        filename = sys.argv[1]

        #now, read all of the data in the file.
        data = read_data(filename)

        #Get all of the remaining arguments
        cmds = sys.argv[2:]


        #clean up the last row in case it didn't finish printing.
        first_row_length = len(data[0])
        last_row_length = len(data[-1])
        if last_row_length < first_row_length:
            data.pop()



        t_data = [float(row[TIME_COLUMN]) for row in data]

        if '-x' in cmds or '-y' in cmds or '-z' in cmds or '-k' in cmds or '-3D' in cmds:

            #parse x, y, z and x, y, z, setpoint data (at minimum. may parse more later)
            # Convert necessary columns to float
            x_data, y_data, z_data = zip(*[(float(row[X_COLUMN]), float(row[Y_COLUMN]), float(row[Z_COLUMN])) for row in data])
            x_set, y_set, z_set = zip(*[(float(row[X_SET_COLUMN]), float(row[Y_SET_COLUMN]), float(row[Z_SET_COLUMN])) for row in data])
            if '-k' in cmds:
                x_kf, y_kf, z_kf = zip(*[(float(row[X_KF_COLUMN]), float(row[Y_KF_COLUMN]), float(row[Z_KF_COLUMN])) for row in data])

        if '-f' in cmds:
            freq_data = [float(row[FREQ_COLUMN]) for row in data]

        if '-fc' in cmds:
            raw_fc_nsec = [float(row[FC_COLUMN]) for row in data]
            fc_array = np.array(raw_fc_nsec, dtype=float)
            fc_array = fc_array/1000000000

        if '-m' in cmds:
            missed_msg_data = [float(row[MISSED_MSG_COLUMN]) for row in data]
            error_data = [float(row[ERROR_COLUMN]) for row in data]

        if '-i' in cmds:
            x_int, y_int, z_int = zip(*[(float(row[X_INTEGRAL]), float(row[Y_INTEGRAL]), float(row[Z_INTEGRAL])) for row in data])

        if '-u' in cmds:
            # print('first -u')
            roll_u, pitch_u, z_u, yaw_u =  zip(*[(float(row[U_ROLL_COLUMN]), float(row[U_PITCH_COLUMN]), float(row[U_Z_COLUMN]), float(row[U_YAW_COLUMN])) for row in data])
            roll_u = np.array(roll_u)
            pitch_u = np.array(pitch_u)
            cmd_pitch = np.array(z_u)

            desired_roll = (roll_u-1500)*(-0.13)
            desired_pitch = (pitch_u-1500)*(-0.13)
            commanded_pitch = (cmd_pitch - 1500)*(-0.13)
            # desired_roll = (-0.1216)*roll_u + 180
            # desired_pitch = (-0.1216)*pitch_u + 180

            pitch_data, roll_data, yaw_data = zip(*[(float(row[13]), float(row[14]), float(row[15])) for row in data])
            pitch_data = np.array(pitch_data)
            pitch_data = pitch_data*180/np.pi
            roll_data = np.array(roll_data)
            roll_data = roll_data*180/np.pi


        

        
 
        #check for a -e flag to plot error graphs instead of pos and setpoint graphs
        do_error = False
        if '-e' in cmds:
            do_error = True

        do_kalman_filter_comparison = False
        if '-k' in cmds:
            do_kalman_filter_comparison = True

        

        #Get any start and end time limints
        data_start_index = 0
        if '-s' in cmds:
            index_of_start = cmds.index('-s')
            start_time = float(cmds[index_of_start + 1])

            for i,t in enumerate(t_data):
                if t > start_time:
                    data_start_index = i
                    break
        
        data_end_index = False
        if '-d' in cmds:
            index_of_end = cmds.index('-d')
            end_time = float(cmds[index_of_end + 1])

            for i,t in enumerate(t_data):
                if t < end_time:
                    continue
                else:
                    data_end_index = i - 1
                    break

            x_data = x_data[data_start_index:data_end_index]
            y_data = y_data[data_start_index:data_end_index]
            z_data = z_data[data_start_index:data_end_index]
            t_data = t_data[data_start_index:data_end_index]
            x_set = x_set[data_start_index:data_end_index]
            y_set = y_set[data_start_index:data_end_index]
            z_set = z_set[data_start_index:data_end_index]
            x_kf = x_kf[data_start_index:data_end_index]
            y_kf = y_kf[data_start_index:data_end_index]
            z_kf = z_kf[data_start_index:data_end_index]
            


        for i,cmd in enumerate(cmds):

            # print(i,cmd)

            #Plot x
            if cmd == '-x':
                error = np.array(x_data) - np.array(x_set)
                if do_error:
                    x_fig, x_ax = plot_2D(t_data, error)
                else:
                    x_fig, x_ax = plot_2D(t_data, x_data, label ='actual pos')
                    x_ax.scatter(t_data,x_set, label = 'desired pos', s=2)
                x_ax.set_xlabel('time')
                x_ax.set_ylabel('x')
                print('Average x error: ' + str(np.average(abs(error))))

            if cmd == '-y':
                error = np.array(y_data) - np.array(y_set)
                if do_error:
                    y_fig, y_ax = plot_2D(t_data, error)
                else:
                    y_fig, y_ax = plot_2D(t_data, y_data, label='actual pos')
                    y_ax.scatter(t_data,y_set, label = 'desired pos', s=2)
                y_ax.set_xlabel('time')
                y_ax.set_ylabel('y')
                print('Average y error: ' + str(np.average(abs(error))))

            if cmd == '-z':
                error = np.array(z_data) - np.array(z_set)
                if do_error:
                    z_fig, z_ax = plot_2D(t_data, error)
                else:
                    z_fig, z_ax = plot_2D(t_data, z_data, label='actual pos')
                    z_ax.scatter(t_data,z_set, label = 'desired pos', s=2)
                z_ax.set_xlabel('time')
                z_ax.set_ylabel('z')
                print('Average z error: ' + str(np.average(abs(error))))

            # # voltage 
            # if cmd == '-v':
            #     v_fig, v_ax = plot_2D(t_data, v_data, label='actual pos')
            #     v_ax.set_xlabel('time')
            #     v_ax.set_ylabel('voltage')

            # # power
            # if cmd == '-p':
            #     p_fig, p_ax = plot_2D(t_data, p_data, label='actual pos')
            #     p_ax.set_xlabel('time')
            #     p_ax.set_ylabel('power')
            
            # # current 
            # if cmd == '-C':
            #     p_fig, p_ax = plot_2D(t_data, tuple(x / y for x, y in zip(p_data, v_data)), label='actual pos')
            #     p_ax.set_xlabel('time')
            #     p_ax.set_ylabel('current')

            #Custom plot (columns specified)
            if cmd == '-c':
                index_of_dash_c = i
                print(index_of_dash_c)
                index_of_x = int(cmds[index_of_dash_c + 1])
                index_of_y = int(cmds[index_of_dash_c + 2])
                plot_2_y = False
                try:
                    y2_cmd = cmds[index_of_dash_c + 3]
                    if '-' in y2_cmd:
                        plot_2_y = False
                    else:
                        index_of_y2 = int(cmds[index_of_dash_c + 2])
                        plot_2_y = True

                except IndexError:
                    pass

                if not plot_2_y:
                    x,y = zip(*[(float(row[index_of_x]), float(row[index_of_y])) for row in data])               
                    print(x[0:10])
                    print(y[0:10])
                    c_fig, c_ax = plot_2D(x,y)
                elif plot_2_y:
                    x,y,y2 = zip(*[(float(row[index_of_x]), float(row[index_of_y]), float(row[index_of_y2])) for row in data])               
                    print(x[0:10])
                    print(y[0:10])
                    c_fig, c_ax = plot_2D(x,y)
                    c_ax.scatter(x,y2,s=1)

            if cmd == '-3D':
                fig, ax = plot_3d_scatter(data_start_index, data_end_index, x_data, y_data, z_data, x_set, y_set, z_set)

            if cmd == '-k':


                fig_x = plt.figure()
                ax_x = fig_x.add_subplot()
                ax_x.scatter(t_data, x_data, label = 'real', s=4)
                ax_x.scatter(t_data, x_kf, label = 'estimate', s=2)
                ax_x.set_xlabel('t')
                ax_x.set_ylabel('x')
                ax_x.legend()

                fig_y = plt.figure()
                ax_y = fig_y.add_subplot()
                ax_y.scatter(t_data, y_data, label = 'real', s=4)
                ax_y.scatter(t_data, y_kf, label = 'estimate', s=2)
                ax_y.set_xlabel('t')
                ax_y.set_ylabel('y')
                ax_y.legend()

                fig_z = plt.figure()
                ax_z = fig_z.add_subplot()
                ax_z.scatter(t_data, z_data, label = 'real', s=4)
                ax_z.scatter(t_data, z_kf, label = 'estimate', s=2)
                ax_z.set_xlabel('t')
                ax_z.set_ylabel('z')
                ax_z.legend()


                x_error = np.array(x_kf) - x_data
                y_error = np.array(y_kf) - y_data
                z_error = np.array(z_kf) - z_data

                fig_e = plt.figure()
                ax_e = fig_e.add_subplot()
                ax_e.scatter(t_data, x_error, label = 'x', s=2)
                ax_e.scatter(t_data, y_error, label = 'y', s=2)
                ax_e.scatter(t_data, z_error, label = 'z', s=2)
                ax_e.set_xlabel('t')
                ax_e.set_ylabel('kf error')
                ax_e.legend()

            if cmd == '-f':
                temp_freq_data = []
                for i,item in enumerate(freq_data):
                    if item > 500:
                        temp_freq_data.append(500)
                    else:
                        temp_freq_data.append(item)
                q_fig, ax_q = plot_2D(t_data, temp_freq_data, label='freq')
                ax_q.set_xlabel('t')
                ax_q.set_ylabel('freq')
                print('Average Freq:' + str(np.mean(freq_data)), np.max(freq_data), np.min(freq_data))
                print('Median Freq: ' + str(np.median(freq_data)))
            
            if cmd == '-m':
                m_fig, ax_m = plot_2D(t_data, missed_msg_data, label='msg count diff')
                ax_m.set_xlabel('t')
                ax_m.set_ylabel('count difference')

            if cmd == '-fc':
                m_fig, ax_m = plot_2D(t_data, fc_array, label='time since last fc cmd')
                ax_m.set_xlabel('t')
                ax_m.set_ylabel('time (s)')

            if cmd == '-i':

                fig_xi = plt.figure()
                ai_x = fig_xi.add_subplot()
                ai_x.scatter(t_data, x_int, label = 'integral', s=4)
                ai_x.set_xlabel('t')
                ai_x.set_ylabel('x')
                ai_x.legend()

                fig_yi = plt.figure()
                ai_y = fig_yi.add_subplot()
                ai_y.scatter(t_data, y_int, label = 'integral', s=4)
                ai_y.set_xlabel('t')
                ai_y.set_ylabel('y')
                ai_y.legend()

                fig_zi = plt.figure()
                ai_z = fig_zi.add_subplot()
                ai_z.scatter(t_data, z_int, label = 'integral', s=4)
                ai_z.set_xlabel('t')
                ai_z.set_ylabel('z')
                ai_z.legend()

            if cmd == '-u':
                # print('doing the u')
                x_fig, x_ax = plot_2D(t_data, pitch_data, label ='actual pitch')
                x_ax.scatter(t_data, desired_pitch, label = 'desired pitch', s=2)
                # x_ax.scatter(t_data, commanded_pitch, label = 'pitch cmdd', s=2, c='black')
                x_ax.set_xlabel('time')
                x_ax.set_ylabel('pitch (deg)')

                y_fig, y_ax = plot_2D(t_data, roll_data, label ='actual roll')
                y_ax.scatter(t_data, desired_roll, label = 'desired roll', s=2)
                y_ax.set_xlabel('time')
                y_ax.set_ylabel('roll (deg)')
                


        plt.show()
    except KeyboardInterrupt:
        return
        

def read_data(csv_file):
    with open(csv_file,newline='') as file:
        reader = csv.reader(file)
        data = list(reader)[NUMBER_OF_HEADER_ROWS:]  #Skip the first N rows

    return data

def plot_2D(x,y, label = False):
    fig = plt.figure()
    ax = fig.add_subplot()
    if label:
        ax.scatter(x,y, label = label, s=2)
    else:
        ax.scatter(x,y, label = label, s=2)
    # ax.set_aspect('equal')
    return fig, ax



def plot_3d_scatter(s, e, x1, y1, z1, x2, y2, z2):
    
    # Create a 3D figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    if e:
        # Scatter plot 1
        ax.scatter(x1[s:e], y1[s:e], z1[s:e], c='b', marker='o', label='Actual Data', s=2)
        
        # Scatter plot 2
        ax.scatter(x2[s:e], y2[s:e], z2[s:e], c='r', marker='^', label='Desired Pos', s=2)
    else:
        # Scatter plot 1
        ax.scatter(x1, y1, z1, c='b', marker='o', label='Actual Data', s=2)
        
        # Scatter plot 2
        ax.scatter(x2, y2, z2, c='r', marker='^', label='Desired Pos', s=2)
    
    
    # Labels and legend
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.legend()
    
    # Make the plot square
    ax.set_box_aspect([1, 1, 1])
    
    return fig, ax


    
# plot_3d_scatter('control_2025-01-29-145855.csv')
if __name__ == "__main__":
    primary()
