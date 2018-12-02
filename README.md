# PowertrainHybridVehicleTraffic
Energy-efficient powertrain control of Hybrid Vehicles through traffic jams

## Introduction

- Hybrid Electric Vehicles contributes to fuel savings and emission reduction aims, which ultimately reduces energy consumption. The aim of this project is to develop efficient powertrain control for hybrid vehicles to reduce energy consumption during heavy traffic. The approach is to reduce fuel consumption by switching between electric drive and internal combustion engine during traffic scenario. The change in acceleration helps to find the velocity which in turn determines the position of the subject vehicle which is also dependent on the vehicle ahead of the subject vehicle. This calculation considers the efficiency of electric drive and internal combustion engine as well at different velocities and done.
-The PDE traffic model is the one which is utilized. The leading vehicle and following vehicle have an initial set of conditions which they are following and then the traffic model leads the following transitions. There is a range of safe distance which is always maintained between the 2 vehicles and the leading vehicle parameters are used to derive the following vehicle parameters like drive cycle and mass fuel consumption.

## Instructions to run the code
- From this directory, first run the file **"DO_Project_Code.m"** file which will give drive profile of our vehicle. This output file would be named as "cyc_mph.m". Output file will have a matrix 'cyc_mph' which will have two columns, first for time and second for optimized velocity of our vehicle.

We are using traffic flow model with safe distance concept to optimize velocity of our vehicle with respect to velocity of leading vehicle in the traffic. Random numbers are used in PDE model of traffic to generate the velocity of leading vehicle at different time steps.

- Then run **"fMPG.m"** file which needs cycle number as input. The cycle we got above is setted as cycle 4.

This will give us plots for SOC and Fuel Consumption.
