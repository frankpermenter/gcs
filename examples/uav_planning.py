#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import os
import shutil
import pickle
import time
import pprint
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from IPython.display import SVG

from pydrake.examples.quadrotor import QuadrotorGeometry
from pydrake.geometry import MeshcatVisualizerCpp, Rgba, StartMeshcat
from pydrake.geometry.optimization import HPolyhedron, VPolytope
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.perception import PointCloud
#from pydrake.solvers.gurobi import GurobiSolver
from pydrake.solvers.conex import ConexSolver
#from pydrake.solvers.mosek import MosekSolver
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

from gcs.bezier import BezierGCS
from gcs.rounding import *
from reproduction.uav.helpers import FlatnessInverter
from reproduction.uav.building_generation import *
from reproduction.util import *

#g_lic = GurobiSolver.AcquireLicense()
#m_lic = MosekSolver.AcquireLicense()


# In[ ]:


# Start the visualizer (run this cell only once, each instance consumes a port)
#meshcat = StartMeshcat()
#
#meshcat.SetProperty("/Grid", "visible", False)
#meshcat.SetProperty("/Axes", "visible", False)
#meshcat.SetProperty("/Lights/AmbientLight/<object>", "intensity", 0.8)
#meshcat.SetProperty("/Lights/PointLightNegativeX/<object>", "intensity", 0)
#meshcat.SetProperty("/Lights/PointLightPositiveX/<object>", "intensity", 0)


# # Generate Building and Plan Through

# In[ ]:


start = np.array([-1, -1])
goal = np.array([2, 1])
building_shape = (3, 3)
start_pose = np.r_[(start-start)*5, 1.]
goal_pose = np.r_[(goal-start)*5., 1.]

# Generate a random building
np.random.seed(3)
grid, outdoor_edges, wall_edges = generate_grid_world(shape=building_shape, start=start, goal=goal)
regions = compile_sdf(FindModelFile("models/room_gen/building.sdf"), grid, start, goal, outdoor_edges, wall_edges)


# In[ ]:


# Build the GCS optimization
print("HEHEHE")
gcs = BezierGCS(regions, order=7, continuity=4, hdot_min=1e-3, full_dim_overlap=True)
print("DONE HEHEHE")

gcs.addTimeCost(1e-3)
gcs.addPathLengthCost(1)
gcs.addVelocityLimits(-10 * np.ones(3), 10 * np.ones(3))
regularization = 1e-3
gcs.addDerivativeRegularization(regularization, regularization, 2)
gcs.addDerivativeRegularization(regularization, regularization, 3)
gcs.addDerivativeRegularization(regularization, regularization, 4)
gcs.addSourceTarget(start_pose, goal_pose, zero_deriv_boundary=3)

gcs.setPaperSolverOptions()
gcs.setSolver(ConexSolver())
gcs.setRoundingStrategy(randomForwardPathSearch, max_paths=10, max_trials=100, seed=0)

# Solve GCS
print("HEHEH")
trajectory = gcs.SolvePath(True, verbose=False, preprocessing=True)[0]




