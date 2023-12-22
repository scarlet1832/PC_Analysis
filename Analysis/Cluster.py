from sklearn.mixture import GaussianMixture as GMM
from sklearn.manifold import TSNE
from sklearn.linear_model import RANSACRegressor
from sklearn.cluster import KMeans
from sklearn.cluster import DBSCAN
from sklearn.decomposition import PCA
import numpy as np
import pandas as pd
from sklearn.neighbors import NearestNeighbors
import math,os
from matplotlib.pyplot import MultipleLocator
import matplotlib.pyplot as plt

def filter_background(df,net_interval=0.02,min_points=10,plane_min_dist=0.035):
    # df_near=df.loc[df["z"]<=100]
    df_near=df
    if df.shape[0]<min_points:
        return df
    # if df["z"].max()>100:
    #     min_points=5
    net_num=math.ceil((df_near["x"].max()-df_near["x"].min())/net_interval)
    hist,bin_edges=np.histogram(df_near["x"].to_numpy(),bins=max(net_num,2))
    below_zero_idx=bin_edges[bin_edges<0].shape[0]
    hist=hist[:min(below_zero_idx,9)]
    if hist.max()<min_points:
        return df
    else:
        low=bin_edges[np.argmax(hist)]
        up=bin_edges[np.argmax(hist)+1]
        df_plane=df_near.loc[(df_near["x"]>=low)&(df_near["x"]<=up)]
        temp=plane_pca(df_plane.loc[:,["x","y","z"]].to_numpy())
        df=df.loc[np.abs(df.loc[:,["x","y","z"]].to_numpy()@temp[:3]+temp[3])>plane_min_dist]
        return df
    

def plane_pca(data):
    pca=PCA(n_components=len(data[0]))
    pca.fit(np.array(data))
    vector = pca.components_[-1]
    vector/=np.linalg.norm(vector)
    d = -vector.dot(pca.mean_)
    temp = np.r_[vector,d]
    return temp


def plot3d(df,label_sorted,max_distance):
    x_major_locator = MultipleLocator(0.2)
    y_major_locator = MultipleLocator(0.2)
    z_major_locator = MultipleLocator(0.2)
    for idx,i in enumerate(label_sorted):
        fig=plt.figure(figsize=(20,10))
        ax=fig.add_subplot(111, projection='3d')
        data=df.loc[df["label"]==i,["x","y","z"]].to_numpy()
        ax.plot(data[:,0],data[:,1],data[:,2],".")
        ax.xaxis.set_major_locator(x_major_locator)
        ax.yaxis.set_major_locator(y_major_locator)
        ax.zaxis.set_major_locator(z_major_locator)
        ax.view_init(elev=-39, azim=3)
        # plt.show()
        plt.savefig(f"./figure/{max_distance}m_{idx}.jpg")
        plt.close()
        
def plot3d_one_fig(df,label_sorted,figure_path):
    fig=plt.figure()
    plt.rcParams['axes.facecolor'] = 'black'
    ax=fig.add_subplot(111, projection='3d')
    for idx,i in enumerate(label_sorted):
        data=df.loc[df["label"]==i,["x","y","z"]].to_numpy()
        ax.plot(data[:,0],data[:,1],data[:,2],".")
    ax.view_init(elev=-62, azim=0)
    plt.axis('off')  #不显示坐标轴
    # plt.xticks([])
    # plt.yticks([])
    # ax.zaxis.set_ticklabels([])
    try:
        if len(df):
            yspan = max(df['y']) - min(df['y'])
            xspan = (max(df['x']) - min(df['x'])) / yspan
            zspan = (max(df['z']) - min(df['z'])) / yspan
            ax.set_box_aspect((round(xspan, 1), 1, round(zspan, 1)))
        else:
            ax.set_box_aspect((1, 1, 1))
    except AttributeError:
        plt.gca().set_aspect('equal')
    # plt.show()
    if not os.path.exists(os.path.dirname(figure_path)):
        os.makedirs(os.path.dirname(figure_path))
    plt.savefig(figure_path,bbox_inches='tight', pad_inches=0)  
    plt.close()


class Cluster(object):
    def __init__(self,df) -> None:
        self.df=df
        
    def cluster(self):
        #先聚成两类，将车识别出一类，再对剩余聚类
        xyz=self.df[["y","z"]].to_numpy()
        kmeans_model=KMeans(n_clusters=2)
        kmeans_model.fit(xyz)
        label1=kmeans_model.labels_
        if self.df.loc[label1==0]["z"].max()>self.df.loc[label1==1]["z"].max():
            label1[label1==0]=-1
        else:
            label1[label1==1]=-1
        xyz=self.df.loc[label1!=-1,["y"]].to_numpy()
        if xyz.shape[0]>600:
            dbscan=DBSCAN(eps=0.1,min_samples=2)
        elif xyz.shape[0]>100:
            dbscan=DBSCAN(eps=0.16,min_samples=2)
        elif xyz.shape[0]>50:
            dbscan=DBSCAN(eps=0.2,min_samples=2)
        else:
            dbscan=DBSCAN(eps=0.5,min_samples=2)
        dbscan.fit(xyz)
        label2=dbscan.labels_
        label2-=label2.min()
        label1[label1!=-1]=label2
        label1[label1==-1]=np.unique(label2).shape[0]
        self.df.insert(self.df.shape[1],"label",label1)
        last_shape=self.df.loc[self.df["label"]==self.df["label"].max()].shape[0]
        while 1:
            self.filter_vehicle(np.unique(label2).shape[0])
            current_shape=self.df.loc[self.df["label"]==self.df["label"].max()].shape[0]
            if last_shape==current_shape:
                break
            else:
                last_shape=current_shape
        self.filter_extral_class()
        self.filter_background()
        return self.df

    def filter_vehicle(self,label,radius=1.8,min_num=5):
        df_vehicle=self.df.loc[self.df["label"]==label]
        if df_vehicle.shape[0]<min_num+3:
            return
        if df_vehicle.shape[0]>300:
            radius=0.2
            min_num=12
        elif df_vehicle.shape[0]>200:
            radius=0.3
            min_num=8
        data=df_vehicle[["y","z"]].to_numpy()
        neigh = NearestNeighbors(radius=radius)
        neigh.fit(data)
        indices = neigh.radius_neighbors(data, return_distance=False)
        delete_points=[]
        for i,neighbors in enumerate(indices):
            if len(neighbors)<=min_num:
                delete_points.append(df_vehicle.iloc[i].name)
        self.df=self.df.drop(index=delete_points)        


    def filter_extral_class(self):
        if np.unique(self.df["label"]).shape[0]<=5:
            return
        min_num=2
        for label in np.unique(self.df["label"]):
            A=self.df.loc[self.df["label"]==label]
            if A.shape[0]<=min_num:
                self.df=self.df.drop(A.index)

    def filter_background(self):
        df_ouput=pd.DataFrame(columns=self.df.columns)
        for i in np.unique(self.df["label"]):
            one_class=self.df.loc[self.df["label"]==i]
            # if i==self.df["label"].max():
            #     one_class=filter_background(one_class,net_interval=0.05,plane_min_dist=0.03)
            # else:
            one_class=filter_background(one_class)
            df_ouput=pd.concat([df_ouput,one_class],ignore_index=True)
        self.df=df_ouput


def cluster_dbscan(df):
    xyz=df[["y","z"]].to_numpy()
    dbscan=DBSCAN(eps=0.3,min_samples=2)
    dbscan.fit(xyz)
    label2=dbscan.labels_
    label2-=label2.min()
    df.insert(df.shape[1],"label",label2)

    
def cluster_kmeans(df,label_num):
    xyz=df.loc[:,["x","y","z"]].to_numpy()
    kmeans_model=KMeans(n_clusters=2)
    kmeans_model.fit(xyz)
    label1=kmeans_model.labels_
    if df.loc[label1==0]["z"].max()>df.loc[label1==1]["z"].max():
        label1[label1==0]=label_num-1
    else:
        label1[label1==1]=label_num-1
    xyz=df.loc[label1!=label_num-1,["y"]].to_numpy()
    kmeans_model=KMeans(n_clusters=label_num-1)
    kmeans_model.fit(xyz)
    label2=kmeans_model.labels_
    label1[label1!=label_num-1]=label2
    
    df.insert(df.shape[1],"label",label1)
    
    
def cluster_gmm(df,label_num):
    #先聚成两类，将车识别出一类，再对剩余聚类
    xyz=df.loc[:,["y","z"]].to_numpy()
    gmm_model=GMM(n_components=2,covariance_type='diag',random_state=0)
    gmm_model.fit(xyz)
    label1=gmm_model.predict(xyz)
    if df.loc[label1==0]["z"].max()>df.loc[label1==1]["z"].max():
        label1[label1==0]=label_num-1
    else:
        label1[label1==1]=label_num-1
    xyz=df.loc[label1!=label_num-1,["y"]].to_numpy()
    gmm_model=GMM(n_components=label_num-1,covariance_type='diag')
    gmm_model.fit(xyz)
    label2=gmm_model.predict(xyz)
    label1[label1!=label_num-1]=label2
    
    df.insert(df.shape[1],"label",label1)

def cluster_kmeans_gmm(df,label_num):
    #先聚成两类，将车识别出一类，再对剩余聚类
    xyz=df.loc[:,["y","z"]].to_numpy()
    kmeans_model=KMeans(n_clusters=2)
    kmeans_model.fit(xyz)
    label1=kmeans_model.labels_
    if df.loc[label1==0]["z"].max()>df.loc[label1==1]["z"].max():
        label1[label1==0]=label_num-1
    else:
        label1[label1==1]=label_num-1
    xyz=df.loc[label1!=label_num-1,["y"]].to_numpy()
    gmm_model=GMM(n_components=label_num-1,covariance_type='diag')
    gmm_model.fit(xyz)
    label2=gmm_model.predict(xyz)
    label1[label1!=label_num-1]=label2
    
    df.insert(df.shape[1],"label",label1)
