function [FC_gram, mpg] = fMPG(cycle)
    
    mph2mps = 0.44704;
    g2gallon=1/841.4/3.785;
    
    % select cycle file
    % cycle = 2;
    switch cycle
        case 1     
            cyclename = 'CYC_UDDS';
        case 2
            cyclename = 'CYC_HWFET';
        case 3
            cyclename = 'CYC_US06';
        case 4
            cyclename = 'cyc_mph'; % My cycle
        otherwise
            error('Unsupported Drive Cycle in fMPG.m');
    end
    
    % load selected cycle file
    loadcycle = [cyclename, '.mat'];
    load(loadcycle)
    tcycle = cyc_mph(:,1);
    Vcycle = cyc_mph(:,2)*mph2mps;
    
    % Set simulation parameters
    Vwind = 0; % Wind speed
    alpha_road = 0; % road grade
    Vveh = Vcycle'; % cycle speed as function of time [m/s]
    dt = 1; % simulation time step [sec]
    Vbatt = 350; % battery voltage [V]
    nomcap_Ah = 6.5;        % Nominal capacity [Ah]
    nomcap_As = nomcap_Ah*3600;     % Nominal Capacity [As]
    Mveh = 1.4*1000; % vehicle mass [kg]
    
    % Calculate speed and torque demands AT THE WHEELS for the cycle
    [Wveh, Tveh, Pshft] = hev_vehicle(Vwind, alpha_road, Vveh, Mveh, dt);
    disp('Pareto frontier calculation initiated')

    % Possible engine speed values
    Weng_array = 0:50:4000;
    
    % Possible engine torque values
    Teng_array = [-6 0 10 15 20 25 30 35 40 45 50 55 60 65 70 75 80 85 90 95 100];
    
    % initialize variables
    FC_matrix = zeros(length(Weng_array)*length(Teng_array),length(tcycle));
    Pbatt_matrix = zeros(length(Weng_array)*length(Teng_array),length(tcycle));
    OpPnts = zeros(length(Weng_array)*length(Teng_array),2);
    Wshft = Wveh;
    Tshft = Tveh;
    Vmotor1 = Vbatt;
%     [Ttrans, Wtrans]=hev_finaldrive(Wshft, Tshft);
    
    % Calculate fuel cons. and battery power for all possible engine operating points
    % Outputs: FC_matrix, Pbatt_matrix, OpPnts
    for ww = 1:length(Weng_array)
        for tt = 1:length(Teng_array)
            iter = (ww-1)*length(Teng_array)+tt;
            Teng = Teng_array(tt);
            Weng = Weng_array(ww);
            FC = PoP_engine(Weng, Teng);
            FC = FC+100*(tt==2);
            FC = FC*ones(1,length(tcycle));
            FC_matrix(iter,:) = FC;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % write the code that computes T_motor, w_motor from T_trans
            % W_trans, T_eng, W_eng
            
%             Cmode = eye(2);
%             [Tmg1_comm, Wmg1] = hev_transmission(Wtrans, Weng, Ttrans, Teng, Cmode);
            
            
            Tmg1_comm = 0.5*(Tveh(tt)-Teng);
            Wmg1 = Weng;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            [Imotor1] = PoP_motor42(Tmg1_comm, Wmg1, Vmotor1);
            Ibatt=-Imotor1;
            [Pbatt] = PoP_battery(Vbatt, Ibatt);
            Pbatt_matrix(iter,:) = Pbatt;
            OpPnts(iter,:) = [Weng_array(ww), Teng_array(tt)]; 
        end
    end
    % Set large values for NaN
    FC_matrix(isnan(FC_matrix))=1000;
    Pbatt_matrix(isnan(Pbatt_matrix))=1e6;
    
    % Initialize pareto point variables
    PoP_ID = num2cell(zeros(1,length(tcycle))); % index of pareto points
    PoP_FC = num2cell(zeros(1,length(tcycle))); % Pareto optimal fuel
    PoP_Pbatt = num2cell(zeros(1,length(tcycle))); % Pareto optimal batt pow
    PoP_OpPnts = num2cell(zeros(1,length(tcycle))); % Pareto optimal oper pnts
    
    % Identify pareto optimal points among all operating points
    % Outputs: PoP_ID, PoP_FC, PoP_Pbatt, PoP_OpPnts
    temp = 1:iter;
    for i = 1:length(tcycle)
        if Pshft(i) >=0
            PoP_ID{i} = [];
            PoP_FC{i} = [];
            PoP_Pbatt{i} = [];
            PoP_OpPnts{i} = [];
            X = [FC_matrix(:,i),Pbatt_matrix(:,i)];
            % A fast pareto optimal point detection code
            membership=paretoset(X);
            par_sc = temp(membership>0);
            par_sc = par_sc(length(par_sc):-1:1)';
            PoP_ID{i} = par_sc;
            PoP_FC{i} = FC_matrix(par_sc,i);
            PoP_Pbatt{i} = Pbatt_matrix(par_sc,i);
            PoP_OpPnts{i} = OpPnts(par_sc,:);
        else
            PoP_ID{i} = 0;
            PoP_FC{i} = 0;
            PoP_Pbatt{i} = 0;
            PoP_OpPnts{i} = 0;
        end
    end
    
    % Save pareto points to PoP structure
    PoP{1}.ID = PoP_ID;
    PoP{1}.FC = PoP_FC;
    PoP{1}.Pbatt = PoP_Pbatt;
    PoP{1}.OpPnts = PoP_OpPnts;

    disp('Pareto frontier calculation ended')
    
    %%%%%%% Regerative Energy Mechanism %%%%%%%
    % Identify negative power demands and calculate SOC regenerated
    % Output: brake_index = [braking start time, braking end time, SOC regenerated]
    
    W_br = (Pshft<0).*Wshft;
    T_br = (Pshft<0).*Tshft;
    Weng = 0;
    Teng = 0;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     rewrite this part as line 63
    Tmg1_comm = 0.5*(T_br-Teng);
    Wmg1 = W_br-Weng;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    [Imotor1] = PoP_motor42(Tmg1_comm, Wmg1, Vmotor1);
    Ibatt=-Imotor1;
    [Pbatt] = PoP_battery(Vbatt, Ibatt);
    Ibatt = Pbatt/Vbatt;
    Ibatt(Ibatt<-100) = -100;
    dSOC = -Ibatt*100*dt/nomcap_As;
    start = 0;
    summ = 0;
    brake_index = [];
    for i = 1:length(dSOC)
        if (dSOC(i)> 0)&&(start==0)
            start = i;
        end
        if (start>0)
            summ = summ + dSOC(i);
        end
        if (start>0)&&(dSOC(i)==0)
            brake_index = [brake_index; start-1, i, summ];
            summ = 0;
            start = 0;
        end
    end
    %%%%%%%%%% Regerative Energy Mechanism Ends %%%%%%%%%%%%%%%%%%%%%%
    
    % If architecture cannot regenerate anything, set high fuel cons.
    if isempty(brake_index)
        penalty = 10000;
        FC_gallon = penalty*100;
        FC_gram = FC_gallon/g2gallon;
        mpg = sum(Vcycle)*0.000621371./FC_gallon;
    else
%         brake_index(:,3) = round(10*brake_index(:,3))/10;
%         brake_index(isnan(brake_index)) = 0;
        
        mode_ind = 1;
        mode_type = 1;
        runsim = 1;
        mu = 1;
        % Initial Lambda
        lambda0 = 0.07;
        % Lambda change for iterations
        dlambda = -0.01;
        % Mode shifting penalty (if multi-mode)
        mode_penal = 0.1;
        attempt = 1;
        % Convergenge threshold
        thresh = 0.01;
        ISconverged = 0;
        SOC_des = 60;  % Desired SOC
        while (runsim)
            disp(['ECMS Attempt: ', num2str(attempt)])
            SOC_hist = zeros(1,2);
            mpg_hist = zeros(1,2);
            % Initial two lambda values
            lambda_hist = [lambda0, lambda0+dlambda];
            % Get corresponding SOCs for initial lambdas
            for i=1:2
                lambda = lambda_hist(i);
                [mpg, SOC_fin, FC_plot, SOC_plot, ~, ~, ISfeas] = hybrid_ECMS_ATC(Vcycle, tcycle, brake_index, mode_ind, mode_type, lambda, 0, mode_penal, Mveh, 0, PoP);
                SOC_hist(i) = SOC_fin;
                mpg_hist(i) = mpg;
                
                figure(1);
                hold on;
                plot(FC_plot);
                title("Fuel Consumption throughout the drive");
                xlabel("Time (sec)");
                ylabel("Fuel Consumption (gram/sec)");
                hold off;
                
                figure(2);
                hold on;
                title("State of Charge (SOC) throughout the drive");
                xlabel("Time (sec)");
                ylabel("SOC");
                plot(SOC_plot)
                hold off;
            end
            % SOC error from desired
            SOCerror = (SOC_fin - SOC_des)^2;
            % Start iteration
            iter = 1;
            SOC0 = SOC_fin;
            while(SOCerror >=thresh) && (iter < 20) && (ISfeas)
                
                if SOC_hist(2) == SOC_hist(1)
                    % If two consecutive iterations give same lambdas, add a small perturbation
                    dlambda = dlambda + 0.0001;
                else
                    % Update lambda using secant method
                    dlambda = -(lambda_hist(2)-lambda_hist(1))*(SOC_hist(2)-SOC_des)/(SOC_hist(2)-SOC_hist(1));
                end
                
                lambda = lambda + dlambda;
                % Saturate lambda between 0.9 and 0.001
                lambda = 0.9*(lambda>0.9) + lambda*(lambda<=0.9 & lambda>=0.001) + 0.001*(lambda<0.001);
                % Save the SOC closer to the desired value 
                [~, ind] = min(abs(SOC_hist-SOC_des));
                SOC_hist(1) = SOC_hist(ind);
                lambda_hist(1) = lambda_hist(ind);
                mpg_hist(1) = mpg_hist(ind);
                % Update lambda and SOC far from the desired SOC
                lambda_hist(2) = lambda;
               %[mpg, SOC_fin, FC_plot, SOC_plot, Mode_plot, t_array, ISfeas, Op_sim] = hybrid_ECMS_ATC();
                [mpg, SOC_fin, ~, ~, ~, ~, ISfeas] = hybrid_ECMS_ATC(Vcycle, tcycle, brake_index, mode_ind, mode_type, lambda, mu, mode_penal, Mveh, 0, PoP);
                SOC_hist(2) = SOC_fin;
                mpg_hist(2) = mpg;
                SOCerror = (SOC_fin - SOC_des)^2;
                iter = iter + 1;
            end
            % If not converged, attempt 2 more with different thresh.
            if (iter == 20) && (SOCerror >= thresh)
                 ISconverged = 0;
                 if attempt == 2
                     thresh = 0.04;
                 elseif attempt == 3
                     runsim = 0;
                     [~, ind] = min(abs(SOC_hist-60));
                     SOC_fin = SOC_hist(ind);
                     lambda = lambda_hist(ind);
                     mpg = mpg_hist(ind);
                     % Assume 4% away from desired SOC to be not converged
                     if ((SOC_fin<(SOC_des-4))||(SOC_fin>(SOC_des+4)))
                         disp('ECMS did not converge')
                     else
                         disp('ECMS converged')
                     end
                 end
                 lambda0 = lambda0 + (0.01)*(SOC0 < 45) - (0.01)*(SOC0 > 75);
                 attempt = attempt + 1;
            else
                ISconverged = 1;
                runsim = 0;            
                disp('ECMS converged')
            end
        end
        % Perform mpg correction based on difference between final SOC and desired SOC
        % mpg_new = mpg + lambda*Pbatt_diff
        % where P_batt_diff = K*(SOC_diff)
        infeasible = (mpg ==(-1))| isnan(mpg) | (ISconverged == 0 & ((SOC_fin<(SOC_des-4))|(SOC_fin>(SOC_des+6))));
        mpg(isnan(mpg)) = 1;
        SOC_fin(isnan(SOC_fin)) = 0;
        penalty = infeasible*abs(SOC_des-SOC_fin)*100;
        K = -Vbatt*nomcap_As/(1e5*dt);
        correction = (lambda*K).*(SOC_fin-60)*g2gallon;
        FC_modified = sum(Vcycle)*0.000621371./mpg + correction + penalty;
        % Corrected fuel consumption [g]
        FC_gram = FC_modified/g2gallon;
        % Corrected mpg
        mpg = sum(Vcycle)*0.000621371./FC_modified;
    end
end