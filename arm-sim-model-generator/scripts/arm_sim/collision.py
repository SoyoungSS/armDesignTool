"""Topology-based mating exclusions; never infer exclusions from observed contact."""
import numpy as np

def mating_pairs(n):
    js=n['model']['joints']; pairs=[]
    for i,j in enumerate(js):
        pairs.append({'body1':j['parent'],'body2':j['child'],'reason':'direct joint mating pair'})
        # Zero-offset downstream revolute/fixed frames remain coincident for all q.
        # Their housings share the same installation space as this parent mount.
        for k in range(i+1,len(js)):
            after=js[k]
            if after['type']=='prismatic' or np.linalg.norm(after['origin']['xyz_m'])>1e-10: break
            pairs.append({'body1':j['parent'],'body2':after['child'],'reason':'coincident revolute/fixed installation cluster'})
    return pairs
