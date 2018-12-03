function [mpg, SOC_fin, FC_plot, SOC_plot, Mode_plot, t_array, ISfeas] = hybrid_ECMS_ATC(Vcycle, tcycle, brake_index, mode_ind, mode_type, lambda, mu, mode_penal, Mveh, FR, PoP)

    rpm2rs = pi/30; %(rpm->r/s)
    g2gallon=1/841.4/3.785; %(gram->liters->gallon: diesel)
    soc_des = 60;  % desired SOC
    init_Mode = 1;
    Vbatt = 350; % Battery Voltage
    dt = 1; % simulation time step
    t = 1;
    k = 1;
    
    %%% w/o Pre-Allocation %%%
    t_array = [];
    FC_plot = [];
    SOC_plot = [];
    Op_sim = [];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    nomcap_Ah = 6.5;        % Nominal capacity [Ah]
    nomcap_As = nomcap_Ah*3600;     % Nominal Capacity [As]
    Nmode = length(mode_ind);
    Mode_sim = zeros(1, length(tcycle));
    EFC_sim = zeros(1, length(tcycle));

    FC_sum = 0;
    SOC_sim = soc_des;
    Mode_sim(t) = init_Mode;
    EFC_sim(t) = 0;
    feas_len = 0;
    PoP_IDcell = cell(Nmode,length(tcycle));
    PoP_Pbatt_cell = cell(Nmode,length(tcycle));
    PoP_FC_cell = cell(Nmode,length(tcycle));
    PoP_OpPntscell = cell(Nmode,length(tcycle));
    min_mode = 1;
    for i = 1:Nmode
        feas_len = feas_len + cellfun('length', PoP{i}.ID);
        PoP_IDcell(i,:) = PoP{i}.ID;
        PoP_Pbatt_cell(i,:) = PoP{i}.Pbatt;
        PoP_FC_cell(i,:) = PoP{i}.FC;
        PoP_OpPntscell(i,:) = PoP{i}.OpPnts;
    end
    
    % Feasibilty Check
    feas = find(~feas_len, 1);
    if isempty(feas)
        while t<=length(tcycle)
            p = (SOC_sim<=40)*mu + (SOC_sim>=80)*(-mu);
            EFC_mode = zeros(Nmode,1);
            Op_mode = zeros(Nmode,4);
            dSOC_mode = zeros(Nmode,1);
            FC_mode = zeros(Nmode,1);
            for i = 1:Nmode
                PoPnts = PoP_IDcell{i,t};
               if (sum(PoPnts)>0)
                                       
                    PoP_FC = PoP_FC_cell{i,t};
                    PoP_Pbatt = PoP_Pbatt_cell{i,t};
                    PoP_Ibatt = PoP_Pbatt/Vbatt; %!!!!!
                    PoP_dSOC = -PoP_Ibatt*100*dt/nomcap_As;
                    PoP_EFC = PoP_FC + (PoP_Pbatt/1000)*(lambda*(1+p));
                    [min_EFC, min_I] = min(PoP_EFC);
                    min_FC = PoP_FC(min_I);
                    min_EFC_dSOC = PoP_dSOC(min_I);
                    EFC_mode(i) = min_EFC;
                    FC_mode(i) = min_FC;
                    if mode_type(i) == 1
                        PoP_OpPnts = PoP_OpPntscell{i,t};
                        min_EFC_Op = PoP_OpPnts(min_I,:);
                        [Wveh, Tveh, Pshft] = hev_vehicle(0, 0, Vcycle', Mveh, dt);
                        Wshft = Wveh(t); Tshft = Tveh(t);
                        
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        % write the code that computes T_motor, w_motor from T_trans
                        % W_trans, T_eng, W_eng
                        Tmg = 0.5*Tshft;
                        Wmg = Wshft;
%                         [Ttrans, Wtrans]=hev_finaldrive(Wshft, Tshft);
%                         [Tmg, Wmg] = hev_transmission(Wtrans, Wveh, Ttrans, Tveh, Cmode);
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                        Op_mode(i,:) = [min_EFC_Op, Wmg, Tmg];
                    else
                        PoP_OpPnts = PoP_OpPntscell{i,t};
                        Op_mode(i,:) = [0, 0, PoP_OpPnts'];
                    end
                    dSOC_mode(i) = min_EFC_dSOC;
               else
                    EFC_mode(i) = NaN;
                    Op_mode(i,:) = NaN*ones(1,4);
                    dSOC_mode(i) = NaN;
               end
            end
            EFC_mode(min_mode) = EFC_mode(min_mode) - mode_penal;
            
            [EFC_sim(t), min_mode] = min(EFC_mode);
            Op_opt = Op_mode(min_mode,:);
            dSOC_sim = dSOC_mode(min_mode);
            SOC_old = SOC_sim;
            FC_sim = FC_mode(min_mode);
            FC_sum = FC_sum + FC_sim;
            t_array = [t_array t];
            Mode_sim(t) = min_mode;
            if t==brake_index(k,1)
                t = brake_index(k,2);
                dSOC_sim = dSOC_sim + brake_index(k,3);
                FC_sum = FC_sum + 0.1*(brake_index(k,2)-brake_index(k,1)-1);
                k = k+1 -(k ==size(brake_index,1));

            else
                t = t+1;
            end
            
            SOC_sim = SOC_old + dSOC_sim;
            FC_plot = [FC_plot FC_sum];
            SOC_plot = [SOC_plot SOC_sim];
            %Op_sim = [Op_sim; Op_opt];
        end
        SOC_fin = SOC_sim;
        Mode_plot = Mode_sim(Mode_sim>0);
        FC_gal = FC_sum*g2gallon;
        mpg = sum(Vcycle)*0.000621371/FC_gal;
        ISfeas = 1;
        
    else
        disp(['Infeasible mode selections: ', num2str(mode_ind)])
        mpg = -1;
        SOC_fin = 0;
        ISfeas = 0;
        FC_plot = [0];
        SOC_plot = [0];
        Mode_plot = [0];
        t_array = [0];
    end
end