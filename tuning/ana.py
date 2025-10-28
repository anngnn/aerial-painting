import matplotlib.pyplot as plt
import csv
from mpl_toolkits.mplot3d import Axes3D
import sys
import numpy as np

NUMBER_OF_HEADER_ROWS = 2
X_COLUMN = 7
Y_COLUMN = 8
Z_COLUMN = 9
DX_COLUMN = 10
DY_COLUMN = 11
DZ_COLUMN = 12
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

P_TERM_POS_X = 32
I_TERM_POS_X = 33
D_TERM_POS_X = 34
DESIRED_VEL_X = 35
ERROR_POS_X = 36
ERROR_POS_Y = 37
ERROR_POS_Z = 38

# --- velocity X PID + errors + desired pitch ---
P_TERM_VEL_X = 39
I_TERM_VEL_X = 40
D_TERM_VEL_X = 41
ERROR_VEL_X   = 42
D_ERROR_VEL_X = 43
DESIRED_PITCH_ANGLE = 44  # deg
ACTUAL_VEL_X = 45 

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

        if '-px' in cmds or '-ix' in cmds or '-dx' in cmds or '-vx' in cmds or '-pix' in cmds or '-pdx' in cmds or '-idx' in cmds or '-pidx' in cmds:
            p_term_x = [float(row[P_TERM_POS_X]) for row in data]
            i_term_x = [float(row[I_TERM_POS_X]) for row in data]
            d_term_x = [float(row[D_TERM_POS_X]) for row in data]
            desired_vel_x = [float(row[DESIRED_VEL_X]) for row in data]

        if '-ep' in cmds:
            error_pos_x = [float(row[ERROR_POS_X]) for row in data]
            error_pos_y = [float(row[ERROR_POS_Y]) for row in data]
            error_pos_z = [float(row[ERROR_POS_Z]) for row in data]

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
            
        # Read velocity X PID and velocity errors only if any *x2 flags or -ev2 appear
        if any(flag in cmds for flag in ['-px2','-ix2','-dx2','-pix2','-pdx2','-idx2','-pidx2','-ev2']):
            v_p_x = [float(row[P_TERM_VEL_X]) for row in data]
            v_i_x = [float(row[I_TERM_VEL_X]) for row in data]
            v_d_x = [float(row[D_TERM_VEL_X]) for row in data]
            err_vel_x  = [float(row[ERROR_VEL_X])   for row in data]
            derr_vel_x = [float(row[D_ERROR_VEL_X]) for row in data]

        if '-vxcomp' in cmds:
            actual_vel_x_col = [float(row[DX_COLUMN]) for row in data]
            desired_vel_x_col = [float(row[DESIRED_VEL_X]) for row in data]

        # For desired pitch from CSV (deg) alongside actual pitch in -u or standalone in -dp
        if '-u' in cmds or '-dp' in cmds:
            try:
                desired_pitch_col = np.array([float(row[DESIRED_PITCH_ANGLE]) for row in data])
            except Exception:
                desired_pitch_col = None


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
                x_ax.set_xlabel('Time (s)')
                x_ax.set_ylabel('X Position (m)')
                print('Average x error: ' + str(np.average(abs(error))))

            if cmd == '-y':
                error = np.array(y_data) - np.array(y_set)
                if do_error:
                    y_fig, y_ax = plot_2D(t_data, error)
                else:
                    y_fig, y_ax = plot_2D(t_data, y_data, label='actual pos')
                    y_ax.scatter(t_data,y_set, label = 'desired pos', s=2)
                y_ax.set_xlabel('Time (s)')
                y_ax.set_ylabel('Y Position (m)')
                print('Average y error: ' + str(np.average(abs(error))))

            if cmd == '-z':
                error = np.array(z_data) - np.array(z_set)
                if do_error:
                    z_fig, z_ax = plot_2D(t_data, error)
                else:
                    z_fig, z_ax = plot_2D(t_data, z_data, label='actual pos')
                    z_ax.scatter(t_data,z_set, label = 'desired pos', s=2)
                z_ax.set_xlabel('Time (s)')
                z_ax.set_ylabel('Z Position (m)')
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
                ax_x.set_xlabel('Time (s)')
                ax_x.set_ylabel('X Position (m)')
                ax_x.legend()

                fig_y = plt.figure()
                ax_y = fig_y.add_subplot()
                ax_y.scatter(t_data, y_data, label = 'real', s=4)
                ax_y.scatter(t_data, y_kf, label = 'estimate', s=2)
                ax_y.set_xlabel('Time (s)')
                ax_y.set_ylabel('Y Position (m)')
                ax_y.legend()

                fig_z = plt.figure()
                ax_z = fig_z.add_subplot()
                ax_z.scatter(t_data, z_data, label = 'real', s=4)
                ax_z.scatter(t_data, z_kf, label = 'estimate', s=2)
                ax_z.set_xlabel('Time (s)')
                ax_z.set_ylabel('Z Position (m)')
                ax_z.legend()


                x_error = np.array(x_kf) - x_data
                y_error = np.array(y_kf) - y_data
                z_error = np.array(z_kf) - z_data

                fig_e = plt.figure()
                ax_e = fig_e.add_subplot()
                ax_e.scatter(t_data, x_error, label = 'x', s=2)
                ax_e.scatter(t_data, y_error, label = 'y', s=2)
                ax_e.scatter(t_data, z_error, label = 'z', s=2)
                ax_e.set_xlabel('Time (s)')
                ax_e.set_ylabel('KF Error (m)')
                ax_e.legend()

            if cmd == '-f':
                temp_freq_data = []
                for i,item in enumerate(freq_data):
                    if item > 500:
                        temp_freq_data.append(500)
                    else:
                        temp_freq_data.append(item)
                q_fig, ax_q = plot_2D(t_data, temp_freq_data, label='freq')
                ax_q.set_xlabel('Time (s)')
                ax_q.set_ylabel('Frequency (Hz)')
                print('Average Freq:' + str(np.mean(freq_data)), np.max(freq_data), np.min(freq_data))
                print('Median Freq: ' + str(np.median(freq_data)))
            
            if cmd == '-m':
                m_fig, ax_m = plot_2D(t_data, missed_msg_data, label='msg count diff')
                ax_m.set_xlabel('Time (s)')
                ax_m.set_ylabel('Count Difference')

            if cmd == '-fc':
                m_fig, ax_m = plot_2D(t_data, fc_array, label='time since last fc cmd')
                ax_m.set_xlabel('Time (s)')
                ax_m.set_ylabel('Time (s)')

            if cmd == '-i':

                fig_xi = plt.figure()
                ai_x = fig_xi.add_subplot()
                ai_x.scatter(t_data, x_int, label = 'integral', s=4)
                ai_x.set_xlabel('Time (s)')
                ai_x.set_ylabel('X Integral (m·s)')
                ai_x.legend()

                fig_yi = plt.figure()
                ai_y = fig_yi.add_subplot()
                ai_y.scatter(t_data, y_int, label = 'integral', s=4)
                ai_y.set_xlabel('Time (s)')
                ai_y.set_ylabel('Y Integral (m·s)')
                ai_y.legend()

                fig_zi = plt.figure()
                ai_z = fig_zi.add_subplot()
                ai_z.scatter(t_data, z_int, label = 'integral', s=4)
                ai_z.set_xlabel('Time (s)')
                ai_z.set_ylabel('Z Integral (m·s)')
                ai_z.legend()

            if cmd == '-px':
                # Plot P term only
                fig_p = plt.figure()
                ax_p = fig_p.add_subplot()
                ax_p.scatter(t_data, p_term_x, label='P term', s=2, c='blue')
                ax_p.set_xlabel('Time (s)')
                ax_p.set_ylabel('P Term (m/s)')
                ax_p.legend()
                ax_p.set_title('P Term for X Position')
                print('Average P term: ' + str(np.average(np.abs(p_term_x))))
                
            if cmd == '-ix':
                # Plot I term only
                fig_i = plt.figure()
                ax_i = fig_i.add_subplot()
                ax_i.scatter(t_data, i_term_x, label='I term', s=2, c='orange')
                ax_i.set_xlabel('Time (s)')
                ax_i.set_ylabel('I Term (m/s)')
                ax_i.legend()
                ax_i.set_title('I Term for X Position')
                print('Average I term: ' + str(np.average(np.abs(i_term_x))))
                
            if cmd == '-dx':
                # Plot D term only
                fig_d = plt.figure()
                ax_d = fig_d.add_subplot()
                ax_d.scatter(t_data, d_term_x, label='D term', s=2, c='green')
                ax_d.set_xlabel('Time (s)')
                ax_d.set_ylabel('D Term (m/s)')
                ax_d.legend()
                ax_d.set_title('D Term for X Position')
                print('Average D term: ' + str(np.average(np.abs(d_term_x))))
                
            if cmd == '-vx':
                # Plot desired velocity
                fig_vel = plt.figure()
                ax_vel = fig_vel.add_subplot()
                ax_vel.scatter(t_data, desired_vel_x, label='Desired velocity X', s=2, c='red')
                ax_vel.set_xlabel('Time (s)')
                ax_vel.set_ylabel('Velocity (m/s)')
                ax_vel.legend()
                ax_vel.set_title('Desired X Velocity')
                print('Average desired velocity X: ' + str(np.average(np.abs(desired_vel_x))))
                
            if cmd == '-pix':
                # Plot P and I terms
                fig_pi = plt.figure()
                ax_pi = fig_pi.add_subplot()
                ax_pi.scatter(t_data, p_term_x, label='P term', s=2, c='blue')
                ax_pi.scatter(t_data, i_term_x, label='I term', s=2, c='orange')
                ax_pi.set_xlabel('Time (s)')
                ax_pi.set_ylabel('Control Term Value (m/s)')
                ax_pi.legend()
                ax_pi.set_title('P and I Terms for X Position')
                
            if cmd == '-pdx':
                # Plot P and D terms
                fig_pd = plt.figure()
                ax_pd = fig_pd.add_subplot()
                ax_pd.scatter(t_data, p_term_x, label='P term', s=2, c='blue')
                ax_pd.scatter(t_data, d_term_x, label='D term', s=2, c='green')
                ax_pd.set_xlabel('Time (s)')
                ax_pd.set_ylabel('Control Term Value (m/s)')
                ax_pd.legend()
                ax_pd.set_title('P and D Terms for X Position')
                
            if cmd == '-idx':
                # Plot I and D terms
                fig_id = plt.figure()
                ax_id = fig_id.add_subplot()
                ax_id.scatter(t_data, i_term_x, label='I term', s=2, c='orange')
                ax_id.scatter(t_data, d_term_x, label='D term', s=2, c='green')
                ax_id.set_xlabel('Time (s)')
                ax_id.set_ylabel('Control Term Value (m/s)')
                ax_id.legend()
                ax_id.set_title('I and D Terms for X Position')
                
            if cmd == '-pidx':
                # Plot all PID terms
                fig_pid = plt.figure()
                ax_pid = fig_pid.add_subplot()
                ax_pid.scatter(t_data, p_term_x, label='P term', s=2, c='blue')
                ax_pid.scatter(t_data, i_term_x, label='I term', s=2, c='orange')
                ax_pid.scatter(t_data, d_term_x, label='D term', s=2, c='green')
                ax_pid.set_xlabel('Time (s)')
                ax_pid.set_ylabel('Control Term Value (m/s)')
                ax_pid.legend()
                ax_pid.set_title('PID Terms for X Position')
                
                print('Average P term: ' + str(np.average(np.abs(p_term_x))))
                print('Average I term: ' + str(np.average(np.abs(i_term_x))))
                print('Average D term: ' + str(np.average(np.abs(d_term_x))))

            if cmd == '-ep':
                # Plot position errors
                fig_err = plt.figure()
                ax_err = fig_err.add_subplot()
                ax_err.scatter(t_data, error_pos_x, label='Error X', s=2)
                ax_err.scatter(t_data, error_pos_y, label='Error Y', s=2)
                ax_err.scatter(t_data, error_pos_z, label='Error Z', s=2)
                ax_err.set_xlabel('Time (s)')
                ax_err.set_ylabel('Position Error (m)')
                ax_err.legend()
                ax_err.set_title('Position Errors (XYZ)')
                
                print('Average error X: ' + str(np.average(np.abs(error_pos_x))))
                print('Average error Y: ' + str(np.average(np.abs(error_pos_y))))
                print('Average error Z: ' + str(np.average(np.abs(error_pos_z))))

            if cmd == '-u':
                # Read the PWM command values and actual angles
                roll_u, pitch_u, z_u, yaw_u = zip(*[(float(row[U_ROLL_COLUMN]), float(row[U_PITCH_COLUMN]), float(row[U_Z_COLUMN]), float(row[U_YAW_COLUMN])) for row in data])
                roll_u = np.array(roll_u)
                pitch_u = np.array(pitch_u)
                
                # Convert PWM values to degrees
                # PWM range: 1000-2000, center at 1500
                # If 500 units = -pi radians and -500 units = +pi radians:
                # Then: (pwm - 1500) maps to angle, where 500 → -π and -500 → +π
                # So: angle_rad = -(pwm - 1500) * π/500
                # Or: angle_deg = -(pwm - 1500) * 180/500
                
                desired_roll = -(roll_u - 1500) * 180/500
                desired_pitch = -(pitch_u - 1500) * 180/500
                
                # Convert actual angles from radians to degrees
                pitch_data, roll_data, yaw_data = zip(*[(float(row[13]), float(row[14]), float(row[15])) for row in data])
                pitch_data = np.array(pitch_data) * 180/np.pi
                roll_data = np.array(roll_data) * 180/np.pi

                # Plot pitch comparison
                x_fig, x_ax = plot_2D(t_data, pitch_data, label='actual pitch')
                x_ax.scatter(t_data, desired_pitch, label='desired pitch', s=2)
                x_ax.set_xlabel('Time (s)')
                x_ax.set_ylabel('Pitch (deg)')
                x_ax.legend()

                # Plot roll comparison
                y_fig, y_ax = plot_2D(t_data, roll_data, label='actual roll')
                y_ax.scatter(t_data, desired_roll, label='desired roll', s=2)
                y_ax.set_xlabel('Time (s)')
                y_ax.set_ylabel('Roll (deg)')
                y_ax.legend()

            # --- Velocity X PID terms: single ---
            if cmd == '-px2':
                fig, ax = plot_2D(t_data, v_p_x, label='Vx P')
                ax.set_xlabel('Time (s)'); ax.set_ylabel('Velocity P Term')
                ax.legend(); ax.set_title('Velocity-X: P term')
                print('Avg |Vx P|:', np.average(np.abs(v_p_x)))

            if cmd == '-ix2':
                fig, ax = plot_2D(t_data, v_i_x, label='Vx I')
                ax.set_xlabel('Time (s)'); ax.set_ylabel('Velocity I Term')
                ax.legend(); ax.set_title('Velocity-X: I term')
                print('Avg |Vx I|:', np.average(np.abs(v_i_x)))

            if cmd == '-dx2':
                fig, ax = plot_2D(t_data, v_d_x, label='Vx D')
                ax.set_xlabel('Time (s)'); ax.set_ylabel('Velocity D Term')
                ax.legend(); ax.set_title('Velocity-X: D term')
                print('Avg |Vx D|:', np.average(np.abs(v_d_x)))

            # --- Velocity X PID terms: combos ---
            if cmd == '-pix2':
                fig, ax = plot_2D(t_data, v_p_x, label='Vx P')
                ax.scatter(t_data, v_i_x, label='Vx I', s=2)
                ax.set_xlabel('Time (s)'); ax.set_ylabel('Velocity Terms'); ax.legend()
                ax.set_title('Velocity-X: P & I')

            if cmd == '-pdx2':
                fig, ax = plot_2D(t_data, v_p_x, label='Vx P')
                ax.scatter(t_data, v_d_x, label='Vx D', s=2)
                ax.set_xlabel('Time (s)'); ax.set_ylabel('Velocity Terms'); ax.legend()
                ax.set_title('Velocity-X: P & D')

            if cmd == '-idx2':
                fig, ax = plot_2D(t_data, v_i_x, label='Vx I')
                ax.scatter(t_data, v_d_x, label='Vx D', s=2)
                ax.set_xlabel('Time (s)'); ax.set_ylabel('Velocity Terms'); ax.legend()
                ax.set_title('Velocity-X: I & D')

            if cmd == '-pidx2':
                fig, ax = plot_2D(t_data, v_p_x, label='Vx P')
                ax.scatter(t_data, v_i_x, label='Vx I', s=2)
                ax.scatter(t_data, v_d_x, label='Vx D', s=2)
                ax.set_xlabel('Time (s)'); ax.set_ylabel('Velocity Terms'); ax.legend()
                ax.set_title('Velocity-X: PID')
                print('Avg |Vx P|:', np.average(np.abs(v_p_x)))
                print('Avg |Vx I|:', np.average(np.abs(v_i_x)))
                print('Avg |Vx D|:', np.average(np.abs(v_d_x)))

            # --- Velocity errors (all) ---
            if cmd == '-ev2':
                fig, ax = plot_2D(t_data, err_vel_x, label='error_vel_x')
                ax.scatter(t_data, derr_vel_x, label='d_error_vel_x', s=2)
                ax.set_xlabel('Time (s)'); ax.set_ylabel('Velocity Error (m/s)')
                ax.legend(); ax.set_title('Velocity-X Errors')
                print('Avg |error_vel_x|:', np.average(np.abs(err_vel_x)))
                print('Avg |d_error_vel_x|:', np.average(np.abs(derr_vel_x)))

            # --- Desired pitch from CSV (deg) next to actual pitch (deg) ---
            if cmd == '-dp':
                if 'pitch_data' not in locals():
                    pitch_data = np.array([float(row[13]) for row in data]) * 180/np.pi  # deg

                try:
                    desired_pitch_col = np.array([float(row[DESIRED_PITCH_ANGLE]) for row in data])
                except Exception:
                    desired_pitch_col = None

                if desired_pitch_col is None:
                    print('No desired_pitch_angle column available (-dp).')
                else:
                    # Detect units (control units vs deg)
                    if np.nanmax(np.abs(desired_pitch_col)) <= 520:
                        desired_pitch_units = desired_pitch_col
                    else:
                        desired_pitch_units = -desired_pitch_col * (500.0/90.0)

                    actual_pitch_units = -pitch_data * (500.0/90.0)

                    fig, ax = plot_2D(t_data, actual_pitch_units, label='actual pitch (control units)')
                    ax.scatter(t_data, desired_pitch_units, label='desired pitch (control units)', s=2)
                    ax.set_xlabel('Time (s)')
                    ax.set_ylabel('Pitch (control units)')
                    ax.legend()
                    ax.set_title('Desired vs Actual Pitch (control units)')

                    # --- AUTO ZOOM based on data range ---
                    all_vals = np.concatenate([desired_pitch_units, actual_pitch_units])
                    min_y, max_y = np.nanmin(all_vals), np.nanmax(all_vals)
                    y_margin = 0.1 * (max_y - min_y if max_y != min_y else 10)
                    ax.set_ylim([min_y - y_margin, max_y + y_margin])



            if cmd == '-vxcomp':
                # Negate one of them to match coordinate frames
                fig, ax = plot_2D(t_data, desired_vel_x_col, label='Desired Vel X')
                ax.scatter(t_data, [-v for v in actual_vel_x_col], label='Actual Vel X (dx)', s=2, c='orange')
                # OR alternatively:
                # fig, ax = plot_2D(t_data, [-v for v in desired_vel_x_col], label='Desired Vel X')
                # ax.scatter(t_data, actual_vel_x_col, label='Actual Vel X (dx)', s=2, c='orange')
                ax.set_xlabel('Time (s)')
                ax.set_ylabel('Velocity X (m/s)')
                ax.legend()
                ax.set_title('Desired vs Actual Velocity X')

                


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
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.legend()
    
    # Make the plot square
    ax.set_box_aspect([1, 1, 1])
    
    return fig, ax


    
# plot_3d_scatter('control_2025-01-29-145855.csv')
if __name__ == "__main__":
    primary()