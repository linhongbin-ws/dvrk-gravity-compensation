function data_file_struct = collect_mtm_one_joint(config,...
                                                 mtm_arm,...
                                                 is_collision_checking,...
                                                 is_collecting_data)
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05
%  Copyright (c)  2018, The Chinese University of Hong Kong
%  This software is provided "as is" under BSD License, with
%  no warranty. The complete license can be found in LICENSE

    % Collect Torque data by positive direction
    if config.is_pos_dir & ~is_collision_checking
        data_file_list = collect_mtm_one_joint_with_one_dir(config,...
                                                              mtm_arm,...
                                                              config.pos_joint_range_list,...
                                                              config.pos_data_path,...
                                                              is_collision_checking,...
                                                              is_collecting_data,...
                                                              'positive');
        data_file_struct.pos = data_file_list;
    end
    if config.is_neg_dir & ~is_collision_checking
        data_file_list = collect_mtm_one_joint_with_one_dir(config,...
                                                              mtm_arm,...
                                                              config.neg_joint_range_list,...
                                                              config.neg_data_path,...
                                                              is_collision_checking,...
                                                              is_collecting_data,...
                                                              'negative');
        data_file_struct.neg = data_file_list;
    end
    % Only one direction need to be check while collsition checking
    if is_collision_checking
        data_file_list = collect_mtm_one_joint_with_one_dir(config,...
                                                      mtm_arm,...
                                                      config.neg_joint_range_list,...
                                                      config.neg_data_path,...
                                                      is_collision_checking,...
                                                      is_collecting_data,...
                                                      'positive');
        data_file_struct.pos = data_file_list;
    end
    
end

function data_file_list = collect_mtm_one_joint_with_one_dir(config,...
                                                              mtm_arm,...
                                                              joint_range_list,...
                                                              data_save_path,...
                                                              is_collision_checking,...
                                                              is_collecting_data,...
                                                              dir_name)
 %  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05
                                                          
       arm_name = config.arm_name;
       sample_num = config.sample_num;
       steady_time = config.steady_time;
       data_file_list = {};
       
       if is_collision_checking
            disp(sprintf('Checking trajectory collision for joint#%d by %s direction now..', config.Train_Joint_No,dir_name));
            disp(sprintf('If MTM hit the environment, please hit E-Button to stop instantly!')); 
        end
        for  k = 1:size(joint_range_list,2)
            if config.Train_Joint_No ==1
                Theta = 0;
            else
                Theta = joint_range_list{k}{config.Theta_Joint_No};
                Theta = int32(rad2deg(Theta));
            end
            joint_range = joint_range_list{k};
            [joint_trajectory,jranges_ranges] = generate_joint_grid(joint_range);
            sample_size = size(joint_trajectory,2);
            desired_effort = zeros(7,sample_size, sample_num);
            current_position = zeros(7,sample_size,sample_num);
            
            %Planning the trajectory to pre-plan
            if is_collision_checking
                if (k==1 || k==size(joint_range_list,2))
                    mtm_arm.move_joint(joint_trajectory(:,1));
                    mtm_arm.move_joint(joint_trajectory(:,int32((1+end)/2)));
                    mtm_arm.move_joint(joint_trajectory(:,end));
                end
            end
            if is_collecting_data                   
                disp(sprintf('Start to collect Torque data of Theta Joint, Joint%d, with angle %d by %s direction',config.Theta_Joint_No,...
                                                                                                                   Theta,...
                                                                                                                   dir_name));

                for i=1:sample_size
                    mtm_arm.move_joint(joint_trajectory(:,i));
                    pause(steady_time);
                    for j=1:sample_num
                        [position, velocity, desired_effort(:,i,j)] = mtm_arm.get_state_joint_desired();
                        [current_position(:,i,j), velocity, effort] = mtm_arm.get_state_joint_current();
                    end 
                    disp(sprintf('Moving Train Joint, Joint%d, to angle %d',config.Train_Joint_No, int32(rad2deg(joint_trajectory(config.Train_Joint_No,i)))));
                end
                if exist(data_save_path)~=7
                    mkdir(data_save_path)
                end
                file_str = strcat(data_save_path,'/',sprintf('theta%d',Theta),'.mat');              
                current_date_time =datestr(datetime('now'),'mmmm-dd-yyyy-HH:MM:SS');
                save(file_str,...
                    'joint_trajectory','jranges_ranges','desired_effort',...
                    'current_position','Theta','current_date_time');
                data_file_list{end+1} = file_str;
            end
        end
end
function [joint_trajectory,j_ranges] = generate_joint_grid(joint_range)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

    j_ranges = joint_range;
    [j7,j6,j5,j4,j3,j2,j1] = ndgrid(j_ranges{7},j_ranges{6},j_ranges{5},j_ranges{4},j_ranges{3},j_ranges{2},j_ranges{1});
    joint_trajectory = [j1(:),j2(:),j3(:),j4(:),j5(:),j6(:),j7(:)]';
end
