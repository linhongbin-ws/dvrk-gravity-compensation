function check_dataCollection_info_in_jointlimit(json_file, MTM_Name)
    fid = fopen(json_file);
    if fid<3
        error('cannot open file')
    end
    raw = fread(fid, inf);
    str = char(raw');
    config = jsondecode(str);
    fclose(fid);
    
    %%%%%%%%% Joint 6
    init_q = config.data_collection.joint6.init_joint_range.';
    q_min = init_q;
    q_min(config.data_collection.joint6.Train_Joint_No) = config.data_collection.joint6.train_angle_min;
    q_min(config.data_collection.joint6.Theta_Joint_No) = config.data_collection.joint6.theta_angle_min;
    if ~isWithinJointLimit(q_min, MTM_Name)
        fprintf('fail for q_min when collecting joint6\n')
        return 
    end
    
    q_max = init_q;
    q_max(config.data_collection.joint6.Train_Joint_No) = config.data_collection.joint6.train_angle_max;
    q_max(config.data_collection.joint6.Theta_Joint_No) = config.data_collection.joint6.theta_angle_max;
    if ~isWithinJointLimit(q_max, MTM_Name)
        fprintf('fail for q_max when collecting joint6\n')
        return 
    end
    
    %%%%%%%%% Joint 5
    init_q = config.data_collection.joint5.init_joint_range.';
    q_min = init_q;
    q_min(config.data_collection.joint5.Train_Joint_No) = config.data_collection.joint5.train_angle_min;
    q_min(config.data_collection.joint5.Theta_Joint_No) = config.data_collection.joint5.theta_angle_min;
    if ~isWithinJointLimit(q_min, MTM_Name)
        fprintf('fail for q_min when collecting joint5\n')
        return 
    end
    
    q_max = init_q;
    q_max(config.data_collection.joint5.Train_Joint_No) = config.data_collection.joint5.train_angle_max;
    q_max(config.data_collection.joint5.Theta_Joint_No) = config.data_collection.joint5.theta_angle_max;
    if ~isWithinJointLimit(q_max, MTM_Name)
        fprintf('fail for q_max when collecting joint5\n')
        return 
    end
    
    %%%%%%%%% Joint 4
    init_q = config.data_collection.joint4.init_joint_range.';
    q_min = init_q;
    if strcmp(MTM_Name, 'MTML')
        q_min(config.data_collection.joint4.Train_Joint_No) = config.data_collection.joint4.train_angle_min.MTML;
    else
        q_min(config.data_collection.joint4.Train_Joint_No) = config.data_collection.joint4.train_angle_min.MTMR;
    end
    if ~isWithinJointLimit(q_min, MTM_Name)
        fprintf('fail for q_min when collecting joint4\n')
        return 
    end
    
    q_max = init_q;
    if strcmp(MTM_Name, 'MTML')
        q_max(config.data_collection.joint4.Train_Joint_No) = config.data_collection.joint4.train_angle_max.MTML;
    else
        q_max(config.data_collection.joint4.Train_Joint_No) = config.data_collection.joint4.train_angle_max.MTMR;
    end
    if ~isWithinJointLimit(q_max, MTM_Name)
        fprintf('fail for q_max when collecting joint4\n')
        return 
    end
    
    %%%%%%%%% Joint 3
    if strcmp(MTM_Name, 'MTML')
        init_q = config.data_collection.joint3.init_joint_range.MTML.';
    else
        init_q = config.data_collection.joint3.init_joint_range.MTMR.';
    end    
    q_min = init_q;
    q_min(config.data_collection.joint3.Train_Joint_No) = max(config.data_collection.joint3.train_angle_min,...
                                                              config.data_collection.joint3.couple_lower_limit-config.data_collection.joint3.theta_angle_min);
    q_min(config.data_collection.joint3.Theta_Joint_No) = config.data_collection.joint3.theta_angle_min;
    if ~isWithinJointLimit(q_min, MTM_Name)
        fprintf('fail for q_min when collecting joint3\n')
        return 
    end
    
    q_max = init_q;
    q_max(config.data_collection.joint3.Train_Joint_No) = min(config.data_collection.joint3.train_angle_max,...
                                                          config.data_collection.joint3.couple_upper_limit-config.data_collection.joint3.theta_angle_max);
    q_max(config.data_collection.joint3.Theta_Joint_No) = config.data_collection.joint3.theta_angle_max;
    if ~isWithinJointLimit(q_max, MTM_Name)
        fprintf('fail for q_max when collecting joint3\n')
        return 
    end
    
    %%%%%%%%% Joint 2
    init_q = config.data_collection.joint2.init_joint_range.';
    q_min = init_q;
    q_min(config.data_collection.joint2.Train_Joint_No) = config.data_collection.joint2.train_angle_min;
    if ~isWithinJointLimit(q_min, MTM_Name)
        fprintf('fail for q_min when collecting joint2\n')
        return 
    end
    
    q_max = init_q;
    q_max(config.data_collection.joint2.Train_Joint_No) = config.data_collection.joint2.train_angle_max;
    if ~isWithinJointLimit(q_max, MTM_Name)
        fprintf('fail for q_max when collecting joint2\n')
        return 
    end
    
    %%%%%%%%% Joint 1
    init_q = config.data_collection.joint1.init_joint_range.';
    q_min = init_q;
    if strcmp(MTM_Name, 'MTML')
        train_angle_min = config.data_collection.joint1.train_angle_min.MTML;
    else
        train_angle_min = config.data_collection.joint1.train_angle_min.MTMR;
    end 
    q_min(config.data_collection.joint1.Train_Joint_No) = train_angle_min;
    if ~isWithinJointLimit(q_min, MTM_Name)
        fprintf('fail for q_min when collecting joint1\n')
        return 
    end
    
    q_max = init_q;
    if strcmp(MTM_Name, 'MTML')
        train_angle_max = config.data_collection.joint1.train_angle_max.MTML;
    else
        train_angle_max = config.data_collection.joint1.train_angle_max.MTMR;
    end 
    q_max(config.data_collection.joint1.Train_Joint_No) = train_angle_max;
    if ~isWithinJointLimit(q_max, MTM_Name)
        fprintf('fail for q_max when collecting joint1\n')
        return 
    end
    
    fprintf("Check jointlimit pass! All configurtations are within joint limits.\n")
end