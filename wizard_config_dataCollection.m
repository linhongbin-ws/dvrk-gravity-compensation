function output_file_str = wizard_config_dataCollection(ARM_NAME,...
        SN)
    %  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
    %  Created on: 2018-10-05
    %  Copyright (c)  2018, The Chinese University of Hong Kong
    %  This software is provided "as is" under BSD License, with
    %  no warranty. The complete license can be found in LICENSE
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Initialization%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

    % checking if arguments correct
    argument_checking(ARM_NAME,...
        SN);

    % Read 'dataCollection_configz.json' file
    fid = fopen('dataCollection_config.json');
    if fid<3
        error('cannot open file dataCollection_config.json, please check the path')
    end
    raw = fread(fid, inf);
    str = char(raw');
    config = jsondecode(str);
    fclose(fid);

    % Customized joint pose
    joint_origin_pose = [0,0,0,0,0,0,0];
    joint_tele_pose = [0,0,0,0,90,0,0];
    % Pose of Joint5 that most likely to hit upper plane
    joint_pose_1 = [0,0,0,0,167,0,0];

    % Create a parent data folder for each MTM to store data

    if strcmp(ARM_NAME,'MTML')
        output_data_path_mtm = [config.data_collection.output_data_root_path,'/MTML_',SN];
    else
        output_data_path_mtm = [config.data_collection.output_data_root_path,'/MTMR_',SN];
    end
    mkdir(output_data_path_mtm);

    % Add additional info in config struct and will save to output file
    config.ARM_NAME = ARM_NAME;
    config.SN = SN;

    % Initiate the arm object and move to origin pose
    mtm_arm = mtm(ARM_NAME);
    mtm_arm.move_joint(deg2rad(joint_origin_pose));

    %%%%%%%%%%%%%%%%%%%%%%figure out suitable joint position params for specific system%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if strcmp(ARM_NAME,'MTML')% cover default setting
        config.data_collection.joint6.init_joint_range(1) = 15;
        config.data_collection.joint6.init_joint_range(4) = -110;
    else
        config.data_collection.joint6.init_joint_range(1) = -15;
        config.data_collection.joint6.init_joint_range(4) = 110;
    end
    
    % figure out the upper limit in workspace, when collecting joint #5
    fprintf('setting Joint #5 params\n')
    Moving_Joint_No = 3;
    if strcmp(ARM_NAME,'MTML')% cover default setting
        config.data_collection.joint5.init_joint_range(1) = 15;
        config.data_collection.joint5.init_joint_range(4) = -110;
    else
        config.data_collection.joint5.init_joint_range(1) = -15;
        config.data_collection.joint5.init_joint_range(4) = 110;
    end
    joint_init_pos = config.data_collection.joint5.init_joint_range;
    joint_init_pos(5) = 167 % a joint position that is most likely to hit upper workspace limit
    param_name = 'theta_angle_max';
    default_value = config.data_collection.joint5.theta_angle_max;
    joint_init_pos(3) = default_value-7; 
    goal_msg = 'Moving MTM upward by increasing Joint#3, finish when distal links of MTM 10cm away from top panel of environment';
    

    config.data_collection.joint5.theta_angle_max = wizard_move_one_joint(mtm_arm,...
        joint_init_pos,...
        Moving_Joint_No,...
        param_name,...
        ARM_NAME,...
        default_value,...
        goal_msg);
    
    
    % figure out the upper limit in workspace, when collecting joint #4
    fprintf('setting Joint #4 params\n')
    Moving_Joint_No = 3; %
    if strcmp(ARM_NAME,'MTML')% cover default setting
        config.data_collection.joint4.init_joint_range(1) = 15;
        config.data_collection.joint4.init_joint_range(4) = 90;
    else
        config.data_collection.joint4.init_joint_range(1) = -15;
        config.data_collection.joint4.init_joint_range(4) = -90;
    end
    joint_init_pos = config.data_collection.joint4.init_joint_range;
%     joint_init_pos(4) = 90 % a joint position that is most likely to hit upper workspace limit for train joint
    param_name = 'theta_angle_max';
    default_value = config.data_collection.joint4.theta_angle_max;
    joint_init_pos(3) = default_value-7; %10 degree smaller for saftey reason
    goal_msg = 'Moving MTM upward by increasing Joint#3, finish when distal links of MTM 10cm away from top panel of environment';
    
    config.data_collection.joint4.theta_angle_max = wizard_move_one_joint(mtm_arm,...
        joint_init_pos,...
        Moving_Joint_No,...
        param_name,...
        ARM_NAME,...
        default_value,...
        goal_msg);

    % figure out the front limit in workspace, when collecting joint #3
    fprintf('setting Joint #3 params\n')
    if strcmp(ARM_NAME,'MTML')% cover default setting
        config.data_collection.joint3.init_joint_range.MTML(1) = 15;
        config.data_collection.joint3.init_joint_range.MTML(4) = -110;
    else
        config.data_collection.joint3.init_joint_range.MTMR(1) = -15;
        config.data_collection.joint3.init_joint_range.MTMR(4) = 110;
    end
    if strcmp(ARM_NAME,'MTML')
        joint_init_pos = config.data_collection.joint3.init_joint_range.MTML;
    else
        joint_init_pos = config.data_collection.joint3.init_joint_range.MTMR;
    end
    Moving_Joint_No = 2;
    param_name = 'theta_angle_max';
    default_value = config.data_collection.joint3.theta_angle_max;
    joint_init_pos(2) = default_value-5; %10 degree smaller for saftey reason
    joint_init_pos(3) = config.data_collection.joint3.couple_lower_limit-joint_init_pos(2);
    couple_contraint =  config.data_collection.joint3.couple_lower_limit;
    goal_msg = 'Moving MTM forward, finish when distal links of MTM 10cm away from front panel of environment';
    config.data_collection.joint3.theta_angle_max = wizard_move_two_joint(mtm_arm,...
        joint_init_pos,...
        param_name,...
        ARM_NAME,...
        default_value,...
        goal_msg,...
        couple_contraint);

    % figure out the upper limit in workspace, when collecting joint #3
    if strcmp(ARM_NAME,'MTML')% cover default setting
        config.data_collection.joint3.init_joint_range.MTML(1) = 15;
        config.data_collection.joint3.init_joint_range.MTML(4) = -110;
    else
        config.data_collection.joint3.init_joint_range.MTMR(1) = -15;
        config.data_collection.joint3.init_joint_range.MTMR(4) = 110;
    end
    if strcmp(ARM_NAME,'MTML')
        joint_init_pos = config.data_collection.joint3.init_joint_range.MTML;
    else
        joint_init_pos = config.data_collection.joint3.init_joint_range.MTMR;
    end
    Moving_Joint_No = 3;
    fprintf('setting Joint #3 params\n')
    joint_init_pos(2) = config.data_collection.joint3.theta_angle_max;
    joint_init_pos(3) = config.data_collection.joint3.couple_upper_limit - 10 - joint_init_pos(2);
    param_name = 'couple_upper_limit';
    default_value = config.data_collection.joint3.couple_upper_limit;
    goal_msg = 'Moving MTM upward by increasing Joint#3, finish when distal links of MTM 10cm away from top panel of environment';
    config.data_collection.joint3.couple_upper_limit = wizard_move_one_joint(mtm_arm,...
        joint_init_pos,...
        Moving_Joint_No,...
        param_name,...
        ARM_NAME,...
        default_value,...
        goal_msg,...
        true);

    % figure out the upper limit in workspace, when collecting joint #2
    fprintf('setting Joint #2 params\n')
    if strcmp(ARM_NAME,'MTML')% cover default setting
        config.data_collection.joint2.init_joint_range(1) = 15;
        config.data_collection.joint2.init_joint_range(4) = -110;
    else
        config.data_collection.joint2.init_joint_range(1) = -15;
        config.data_collection.joint2.init_joint_range(4) = 110;
    end
    Moving_Joint_No = 2;
    joint_init_pos = config.data_collection.joint2.init_joint_range;
    param_name = 'train_angle_max';
    default_value = config.data_collection.joint2.train_angle_max;
    joint_init_pos(Moving_Joint_No) = default_value-7;
    goal_msg = 'Moving MTM upward by increasing Joint#2, finish when distal links of MTM 10cm away from top panel of environment';
    config.data_collection.joint2.train_angle_max= wizard_move_one_joint(mtm_arm,...
        joint_init_pos,...
        Moving_Joint_No,...
        param_name,...
        ARM_NAME,...
        default_value,...
        goal_msg);

    if strcmp(ARM_NAME,'MTML')% cover default setting
        config.data_collection.joint1.init_joint_range(4) = -110;
    else
        config.data_collection.joint1.init_joint_range(4) = 110;
    end
   % figure out the left limit in workspace, when collecting joint #1
    if strcmp(ARM_NAME,'MTML')
        fprintf('setting Joint #1 params\n')
        joint_init_pos = config.data_collection.joint1.init_joint_range;
        Moving_Joint_No = 1;
        param_name = 'train_angle_min';
        default_value = config.data_collection.joint1.train_angle_min.MTML;
        goal_msg = 'Moving MTM left by decreasing Joint#1, finish when distal links of MTM 10cm away from left panel of environment';
        config.data_collection.joint1.train_angle_min.MTML = wizard_move_one_joint(mtm_arm,...
            joint_init_pos,...
            Moving_Joint_No,...
            param_name,...
            ARM_NAME,...
            default_value,...
            goal_msg);
        mtm_arm.move_joint(deg2rad(joint_tele_pose));
    else
        fprintf('setting Joint #1 params\n')
        joint_init_pos = config.data_collection.joint1.init_joint_range;

        Moving_Joint_No = 1;
        param_name = 'train_angle_max';
        default_value = config.data_collection.joint1.train_angle_max.MTMR;
        goal_msg = 'Moving MTM right by increasing Joint#1, finish when distal links of MTM 10cm away from right panel of environment';
        config.data_collection.joint1.train_angle_max.MTMR = wizard_move_one_joint(mtm_arm,...
            joint_init_pos,...
            Moving_Joint_No,...
            param_name,...
            ARM_NAME,...
            default_value,...
            goal_msg);
        mtm_arm.move_joint(deg2rad(joint_tele_pose));
    end

    % figure out the right limit in workspace, when collecting joint #1
    if strcmp(ARM_NAME,'MTML')
        fprintf('setting Joint #1 params\n')
        joint_init_pos = config.data_collection.joint1.init_joint_range;
        joint_init_pos(1) = 30;
        Moving_Joint_No = 1;
        param_name = 'train_angle_max';
        default_value = config.data_collection.joint1.train_angle_max.MTML;
        goal_msg = 'Moving MTM right by increasing Joint#1, finish when distal links of MTM 10cm away from right panel of environment/other MTM';
        config.data_collection.joint1.train_angle_max.MTML = wizard_move_one_joint(mtm_arm,...
            joint_init_pos,...
            Moving_Joint_No,...
            param_name,...
            ARM_NAME,...
            default_value,...
            goal_msg);
        mtm_arm.move_joint(deg2rad(joint_tele_pose));
   else
        fprintf('setting Joint #1 params\n')
        joint_init_pos = config.data_collection.joint1.init_joint_range;
        joint_init_pos(1) = -30;
        Moving_Joint_No = 1;
        param_name = 'train_angle_min';
        default_value = config.data_collection.joint1.train_angle_min.MTMR;
        goal_msg = 'Moving MTM left by decreasing Joint#1, finish when distal links of MTM 10cm away from left panel of environment/other MTM';
        config.data_collection.joint1.train_angle_min.MTMR = wizard_move_one_joint(mtm_arm,...
            joint_init_pos,...
            Moving_Joint_No,...
            param_name,...
            ARM_NAME,...
            default_value,...
            goal_msg);
        mtm_arm.move_joint(deg2rad(joint_tele_pose));
    end

    % Final result output display
    close(gcf);

%     fprintf('Your customized parameters is:\n');
%     fprintf('joint3.theta_angle_max: %d\n', config.data_collection.joint3.theta_angle_max);
%     fprintf('joint3.couple_upper_limit: %d\n', config.data_collection.joint3.couple_upper_limit);
%     if strcmp(ARM_NAME,'MTML')
%         fprintf('Joint1.train_angle_min.MTML: %d\n', config.data_collection.joint1.train_angle_min.MTML);
%         fprintf('Joint1.train_angle_max.MTML: %d\n', config.data_collection.joint1.train_angle_max.MTML);
%     end
%     if strcmp(ARM_NAME,'MTMR')
%         fprintf('Joint1.train_angle_min.MTMR: %d\n', config.data_collection.joint1.train_angle_min.MTMR);
%         fprintf('Joint1.train_angle_max.MTMR: %d\n', config.data_collection.joint1.train_angle_max.MTMR);
%     end


    % Execute collision checking process
    collision_checking(config);

    % Save customized json output file for further data_collection process
    output_file_str = [output_data_path_mtm, '/dataCollection_config_customized.json'];
    fid = fopen(output_file_str,'w');
    jsonStr = jsonencode(config);
    fwrite(fid, jsonStr);
    fclose(fid);
    fprintf('Save config file to %s\n', output_file_str);
end

function customized_value = wizard_move_one_joint(mtm_arm,...
        joint_init_pos,...
        Moving_Joint_No,...
        param_name,...
        ARM_NAME,...
        default_value,...
        goal_msg,...
        is_couple_limit)
    input_str = '';
    fprintf('Setting Hard limit for when collecting data of Joint#%d\n', Moving_Joint_No)
    fprintf('Moving to init pose\n');
    mtm_arm.move_joint(deg2rad(joint_init_pos));
    if(~exist('is_couple_limit', 'var'))
        customized_value = joint_init_pos(Moving_Joint_No);
    else
        customized_value = joint_init_pos(Moving_Joint_No) + joint_init_pos(Moving_Joint_No-1);
    end
    joint_pos = joint_init_pos;
    fprintf('Instruction: %s\n', goal_msg);
    fprintf('Arm: %s\n', ARM_NAME);
    fprintf('Moving_Joint_No: %d\n', Moving_Joint_No);
    fprintf('Customized Param Name: %s\n', param_name);
    disp('[i] to increase, [d] to decrease, [r] for recommended, [f] when done');
    fprintf('Recommended value: [%s] = %d degree(s)\n', param_name, default_value);
    lastsize = 0;
    while (true)
        while (~strcmp(input_str,'i') && ~strcmp(input_str,'d') && ~strcmp(input_str,'r') && ~strcmp(input_str,'f'))
            fprintf(repmat('\b', 1, lastsize));
            lastsize = fprintf('Current value: [%s] = %d degree(s)', param_name, customized_value);
            w = waitforbuttonpress;
            if w
                input_str = get(gcf, 'CurrentCharacter');
            end
        end
        if (input_str == 'i')
            customized_value = customized_value + 1;
        elseif (input_str == 'd')
            customized_value = customized_value - 1;
        elseif (input_str == 'r')
            customized_value = default_value;
        else
            fprintf(repmat('\b', 1, lastsize)); % clear last printed line
            break
        end
        if(~exist('is_couple_limit', 'var'))
            joint_pos(Moving_Joint_No) = customized_value;
        else
            joint_pos(Moving_Joint_No) = customized_value - joint_pos(Moving_Joint_No-1);
        end
        mtm_arm.move_joint(deg2rad(joint_pos));
        input_str = '';
    end
end

function customized_value = wizard_move_two_joint(mtm_arm,...
        joint_init_pos,...
        param_name,...
        ARM_NAME,...
        default_value,...
        goal_msg,...
        couple_contraint)
    % Hard Code
    Joint3_pos_min_limit = -35;
    input_str = '';
    %fprintf('Setting Hard limit for when collecting data of Joint#%d\n', 3);
    disp('Moving to init pose');
    mtm_arm.move_joint(deg2rad(joint_init_pos));
    customized_value = joint_init_pos(2);
    joint_pos = joint_init_pos;
    fprintf('Instruction: %s\n', goal_msg);
    fprintf('Arm: %s\n', ARM_NAME);
    fprintf('Moving_Joint_No: %d\n', 2);
    fprintf('Customized Param Name: %s\n', param_name);
    disp('[i] to increase, [d] to decrease, [r] for recommended, [f] when done');
    fprintf('Recommended value: [%s] = %d degree(s)\n', param_name, default_value);
    lastsize = 0;
    while (true)
        while (~strcmp(input_str,'i') && ~strcmp(input_str,'d') && ~strcmp(input_str,'r') && ~strcmp(input_str,'f'))
            fprintf(repmat('\b', 1, lastsize));
            lastsize = fprintf('Current value: [%s] = %d degree(s)', param_name, customized_value);
            w = waitforbuttonpress;
            if w
                input_str = get(gcf, 'CurrentCharacter');
            end
        end
        if (input_str == 'i')
            customized_value = customized_value + 1;
        elseif (input_str == 'd')
            customized_value = customized_value - 1;
        elseif (input_str == 'r')
            customized_value = default_value;
        else
            fprintf(repmat('\b', 1, lastsize)); % clear last printed line
            break
        end
        joint_pos(2) = customized_value;
        joint_pos(3) = couple_contraint - joint_pos(2);
        if joint_pos(3)<= Joint3_pos_min_limit
            customized_value = customized_value-1;
            joint_pos(2) = customized_value;
            joint_pos(3) = couple_contraint - joint_pos(2);
        end
        mtm_arm.move_joint(deg2rad(joint_pos));
        input_str = '';
    end
end

function argument_checking(ARM_NAME,...
        SN)
    if~strcmp(ARM_NAME, 'MTML') && ~strcmp(ARM_NAME, 'MTMR')
        error(['Input of argument ''ARM_NAME''= %s is error, you should input one of the strings ',...
            '[''MTML'',''MTMR'']'], ARM_NAME);
    end
    if ~ischar(SN)
        error('SN input %s is not char array', SN);
    end
end

function collision_checking(config)
    %  Institute: The Chinese University of Hong Kong
    %  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
    %  Created on: 2018-10-05

    % General variables
    ARM_NAME = config.ARM_NAME;
    is_collision_checking = true;
    is_collecting_data = false;

    % Vitual path for collision checking
    input_data_path = 'test';

    % create mtm_arm obj and move every arm in home joint position
    mtm_arm = mtm(ARM_NAME);
    mtm_arm.move_joint([0,0,0,0,0,0,0]);

    config_joint_list = setting_dataCollection(config,...
                                               input_data_path);
    tic;
    current_progress = 0.0;
    total_data_sets = 0;
    for j=1:size(config_joint_list, 2)
        total_data_sets = total_data_sets + config_joint_list{j}.data_size; 
    end
    one_data_progress_increment = 100 / total_data_sets;
    for i = 1:size(config_joint_list, 2)
        collect_mtm_one_joint(config_joint_list{i},...
                              mtm_arm,...
                              is_collision_checking,...
                              is_collecting_data,...
                              current_progress,...
                              one_data_progress_increment);
    end
    mtm_arm.move_joint([0,0,0,0,0,0,0]);

end

