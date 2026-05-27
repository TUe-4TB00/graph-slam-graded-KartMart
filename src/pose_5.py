import numpy as np
from helperfunctions import add_pose_from_global, add_landmark_measurement_from_global
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_pose(graph, initial_estimate, pose_5):
    # Adding the initial estimate for the 5th pose using our helper function `add_pose_from_global` which also adds the odometry factor between X(4) and X(5).
    pose_4 = initial_estimate.atPose2(X(4))
    graph, initial_estimate = add_pose_from_global(
        graph=graph,
        initial_estimate=initial_estimate,
        prev_key=X(4),
        new_key=X(5),
        prev_pose=pose_4,
        new_pose_global=pose_5,
        odom_noise=ODOMETRY_NOISE
    )
    return graph, initial_estimate

def add_landmark_measurement(graph, result, pose_5, landmark):
    # Adding the measurement from X(5) to the chosen landmark using our helper function `add_landmark_measurement_from_global` which calculates the correct bearing and range from the global poses.``
    landmark_point = result.atPoint2(L(landmark))
    graph = add_landmark_measurement_from_global(
        graph=graph,
        pose_key=X(5),
        pose=pose_5,
        landmark_key=L(landmark),
        landmark_point=landmark_point,
        measurement_noise=MEASUREMENT_NOISE
    )
    return graph

def optimize(graph, initial_estimate):
    # TODO: Initialize the optimizer DONE 
    params = gtsam.LevenbergMarquardtParams()
    # Creating the optimizer instance, providing the graph, initial estimate, and parameters.
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate, params)
    # Running the optimization
    result = optimizer.optimize()
    # Print the optimized result


    # TODO: Perform the optimization and print the result

    print("\nFinal Result:\n{}".format(result))

    return result

# def minimize_marginals(graph, initial_estimate, pose_options):
#     #TODO: try different pose and landmark options here, and keep the one with the lowest sum of marginals.
#     best_pose = "a"      # chosen pose option
#     best_landmark = 1    # chosen landmark (1 or 2)
#     pose_5 = pose_options[best_pose]

#     graph, initial_estimate = add_pose(graph, initial_estimate, pose_5) #update my stupid graph and intial estimate


#     result = optimize(graph, initial_estimate) #Optimizing the stupid graph so that the odometries are optimized and my stupid measurements align. this is related to X5

#     graph = add_landmark_measurement(graph, result, pose_5, best_landmark) #Adding a stupid factor between my pose 5 and landmark
    
#     result = optimize(graph, initial_estimate) #W

#     # TODO: Calculate marginal covariances for the relevant variables and visualize the updated factor graph with covariances
#     marginals = gtsam.Marginals(graph, result)
#     sum_of_marginals = marginals.marginalCovariance(L(1)).sum() + marginals.marginalCovariance(L(2)).sum()

#     print("Sum of marginals:", sum_of_marginals)


#     # The sum of the marginals for each landmark can be computed using marginals.marginalCovariance(L(x)).sum()
#     #sum_of_marginals = 0
#     return best_pose, best_landmark, sum_of_marginals

def minimize_marginals(graph, initial_estimate, pose_options):
    #TODO: try different pose and landmark options here, and keep the one with the lowest sum of marginals.
        best_pose = "d"      # chosen pose option
        best_landmark = 1    # chosen landmark (1 or 2)
        pose_5 = pose_options[best_pose]

        graph, initial_estimate = add_pose(graph, initial_estimate, pose_5) #update my stupid graph and intial estimate


        result = optimize(graph, initial_estimate) #Optimizing the stupid graph so that the odometries are optimized and my stupid measurements align. this is related to X5

        graph = add_landmark_measurement(graph, result, pose_5, best_landmark) #Adding a stupid factor between my pose 5 and landmark
        
        result = optimize(graph, initial_estimate) #W

        # TODO: Calculate marginal covariances for the relevant variables and visualize the updated factor graph with covariances
        marginals = gtsam.Marginals(graph, result)
        sum_of_marginals = marginals.marginalCovariance(L(1)).sum() + marginals.marginalCovariance(L(2)).sum()

        print("Sum of marginals:", sum_of_marginals)


        # The sum of the marginals for each landmark can be computed using marginals.marginalCovariance(L(x)).sum()
        #sum_of_marginals = 0
        return best_pose, best_landmark, sum_of_marginals

def minimize_errors(graph, initial_estimate, pose_options):
    #TODO: try different pose and landmark options here, and keep the one with the lowest resulting error.
    best_pose = "b"      # chosen pose option
    best_landmark = 2  # chosen landmark (1 or 2)
    pose_5 = pose_options[best_pose]
    graph, initial_estimate = add_pose(graph, initial_estimate, pose_5)
    result = optimize(graph, initial_estimate)
    graph = add_landmark_measurement(graph, result, pose_5, best_landmark)
    result = optimize(graph, initial_estimate)

    # TODO: create a list of errors (each index corresponds to a pose) and add the error of each pose to the list


    X_1_real = gtsam.Pose2(0, 0, 0)

    X_2_real = gtsam.Pose2(2, 0, 0)

    X_3_real = gtsam.Pose2(4, 0, 0)

    # x_errorx1 = abs(result.atPose2(X(1)).x()-initial_estimate.atPose2(X(1)).x())
    # y_errorx1  = abs(result.atPose2(X(1)).y()-initial_estimate.atPose2(X(1)).y())
    # theta_errorx1  = abs(result.atPose2(X(1)).theta()-initial_estimate.atPose2(X(1)).theta())

    # x_errorx2 = abs(result.atPose2(X(2)).x()-initial_estimate.atPose2(X(2)).x())
    # y_errorx2  = abs(result.atPose2(X(2)).y()-initial_estimate.atPose2(X(2)).y())
    # theta_errorx2  = abs(result.atPose2(X(2)).theta()-initial_estimate.atPose2(X(2)).theta())

    # x_errorx3 = abs(result.atPose2(X(3)).x()-initial_estimate.atPose2(X(3)).x())
    # y_errorx3  = abs(result.atPose2(X(3)).y()-initial_estimate.atPose2(X(3)).y())
    # theta_errorx3  = abs(result.atPose2(X(3)).theta()-initial_estimate.atPose2(X(3)).theta())


    x_errorx1 = abs(result.atPose2(X(1)).x() - X_1_real.x())
    y_errorx1  = abs(result.atPose2(X(1)).y()-X_1_real.y())
    theta_errorx1  = abs(result.atPose2(X(1)).theta()-X_1_real.theta())

    x_errorx2 = abs(result.atPose2(X(2)).x()- X_2_real.x())
    y_errorx2  = abs(result.atPose2(X(2)).y()- X_2_real.y())
    theta_errorx2  = abs(result.atPose2(X(2)).theta()-X_2_real.theta())

    x_errorx3 = abs(result.atPose2(X(3)).x()-X_3_real.x())
    y_errorx3  = abs(result.atPose2(X(3)).y()-X_3_real.y())
    theta_errorx3  = abs(result.atPose2(X(3)).theta()-X_3_real.theta())


    error_total_in_X = x_errorx1 + x_errorx2 + x_errorx3
    error_total_in_Y = y_errorx1 + y_errorx2 + y_errorx3
    error_total_in_theta = theta_errorx1 + theta_errorx2 + theta_errorx3

    error = [error_total_in_X, error_total_in_Y, error_total_in_theta ]
    #THIS GIVES ME ERROR IN X Y AND THETA for pos A and landmark 1

    print("ERROR:", error)
    #list_of_errors = []

    # TODO: compute the sum of the errors and return it along with the best pose and landmark

    sum_of_errors = sum(error)
    print("GCSE",  sum_of_errors)
    return best_pose, best_landmark, sum_of_errors 
