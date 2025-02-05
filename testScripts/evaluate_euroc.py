import argparse
import os
import time

euroc_test_cases = ["V1_01_easy", "V1_02_medium", "V1_03_difficult","V2_01_easy","V2_02_medium","V2_03_difficult"]


ate_result = []
rpe_result = []

def main(config_file_path, datasets_path, gt_path):
    print(config_file_path)
    print(datasets_path)
    for test_case in euroc_test_cases:
        print("test_case: ", test_case) 
        test_case_path = datasets_path + "/" + test_case
        print("test_case_path: ", test_case_path)
        # run ros2 node
        os.system(f"ros2 run delta_vins DeltaVINSTest {config_file_path} --ros-args -p DataSourcePath:={test_case_path} -p TestCaseName:={test_case}")
        # wait for 10 seconds
        time.sleep(2)
        # rename the result file from "TestResults/outputPose.tum" to "TestResults/<test_case>.tum"
        result_file_path = "TestResults/outputPose.tum"
        new_result_file_path = "TestResults/" + test_case + ".tum"
        os.rename(result_file_path, new_result_file_path)

        # run evaluation
        os.system(f"evo_ape euroc {gt_path}/{test_case}.csv TestResults/{test_case}.tum -a > TestResults/{test_case}_ate.txt")
        os.system(f"evo_rpe euroc {gt_path}/{test_case}.csv TestResults/{test_case}.tum -a --delta 5 --delta_unit m > TestResults/{test_case}_rpe.txt")

        # read the result, get the line with contains "rmse"
        with open(f"TestResults/{test_case}_ate.txt", "r") as file:
            for line in file:
                if "rmse" in line:
                    # extract the float value after "rmse" and tab
                    ate_result.append(float(line.split('\t')[1].strip()))

        with open(f"TestResults/{test_case}_rpe.txt", "r") as file:
            for line in file:
                if "rmse" in line:
                    # extract the float value after "rmse" and tab
                    rpe_result.append(float(line.split('\t')[1].strip()))

    # print the results in a table, align the columns
    print(f"{'Test Case':<20} {'ATE':<20} {'RPE':<20}")
    for i in range(len(euroc_test_cases)):
        print(f"{euroc_test_cases[i]:<20} {ate_result[i]:<20} {rpe_result[i]:<20}")





# python3 evaluate_euroc.py -c [config_file_path] -gtp [gt_path]

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config_file", type=str, required=True)
    parser.add_argument("-d", "--datasets_path", type=str, required=True)
    parser.add_argument("-gtp", "--gt_path", type=str, required=True)
    args = parser.parse_args()
    main(args.config_file, args.datasets_path, args.gt_path)
