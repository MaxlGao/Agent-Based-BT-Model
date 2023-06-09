import os
import csv
import multiprocessing
import Config_Max as config
import MultiLaw
import Simulator
import time
import numpy as np

if __name__ == '__main__':
    # Common
    startWith = 0  # Should normally be 0; used for starting part-way into a multisim batch
    sl = config.SimLaw()
    ml = MultiLaw.MultiLaw()
    startTime = time.time()
    y = str(time.gmtime().tm_year)
    M = str(time.gmtime().tm_mon).zfill(2)
    d = str(time.gmtime().tm_mday).zfill(2)
    h = str(time.gmtime().tm_hour).zfill(2)
    m = str(time.gmtime().tm_min).zfill(2)
    s = str(time.gmtime().tm_sec).zfill(2)
    filename = "Megarun_"+y+M+d+'_'+h+m+s+".csv"
    # root_folder = "Output_Media"
    # if not os.path.isdir(root_folder):
    #     os.mkdir(root_folder)
    outputArray = []

    if sl.multiLaw:
        # Setup an array of simlaws unique to each simulation.
        numVariables = ml.__dict__.__len__()  # Number of variables varied
        numValues = []  # Number of unique values of each varied variable.
        varValues = []  # Lists of unique values of each varied variable
        varNames = []  # Names of each varied variable
        combinations = 1
        varInstances = np.ones(numVariables)
        for item in ml.__dict__.items():
            varNames.append(item[0])
            combinations = combinations * len(item[1])
            numValues.append(len(item[1]))
            varValues.append(item[1])
        varInstances = [np.prod(numValues[i + 1:]) for i in range(len(numValues))]
        # Make end of list an int
        varInstances[-1] = 1

        # Start churning out simlaws
        simLaws = [config.SimLaw() for _ in range(combinations)]
        for run in range(combinations):
            code = np.zeros(numVariables)
            simLaws[run].simNumber = run
            n = run
            varValue = []
            for i in range(numVariables):
                # Example:
                # varInstances is [100, 20, 1], meaning Variable A gets repeated 100 times, B 20 times, and C once.
                # Entries 0-99 get the first value of A, 100-199 get the second value, and so on.
                # Entries X00-X19 get the first value of B, X20-X39 get the second value, and so on.
                # Suppose n is 159. First, the number is divided by 100, resulting in 1.59, floored to 1.
                # The remainder, 59, is divided by 20, resulting in 2.95, floored to 2.
                # The remainder, 19, is divided by 1, resulting in 19
                # Simulation 159 uses the second value of A, the third value of B, and the 20th value of C.
                code[i] = np.floor(n / varInstances[i])
                n = n % varInstances[i]
                setattr(simLaws[run], varNames[i], varValues[i][int(code[i])])
                varValue.append(varValues[i][int(code[i])])
            simLaws[run].varValue = varValue

        # Trim according to startWith
        simLaws = simLaws[startWith:]

        # Save the array of simlaws to the original simlaw.
        sl.SLs = simLaws
        print(f"All done setting up for {combinations} simulations.")
        with multiprocessing.Pool() as pool:
            for result in pool.imap(Simulator.run_simulations, simLaws):
                outputArray.append(result)
                with open(filename, 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile, delimiter=',')
                    writer.writerow(result)
                    csvfile.close()
    else:
        # Assume that not using parallel processing == not using multilaw == Running one sim
        result = Simulator.run_simulations(sl)
        outputArray.append(result)
        with open(filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=',')
            writer.writerow(result)
            csvfile.close()
    endTime = time.time()
    print(f"Done with all sims, time taken is {endTime - startTime:7.0f}s")

