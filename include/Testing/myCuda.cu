  #include <thrust/reduce.h>
  #include <thrust/host_vector.h>
  #include <thrust/device_vector.h>

__constant__ double PI = 3.141592653589;

    //  nn              =>    d_nnData        ==> array of nearest point in scene from a given  provenance vector
    //  kp              =>    kpData          ==> key point to tests, 512 * 8 (samples calculated and rotated)
    //  pv              =>    pvData          ==> provenance vectors repited as many as orientations
    //  tp              =>    d_testPoint     ==> testPoint
    //  C               =>    d_C             ==> OUTPUT this will be the result container 
    //  start           =>    start           ==> first number of sample point to test in the batch
    //  end             =>    end             ==> las number of sample point  to test in the batch
    //  weights         =>    wData           ==> provenance vector norms mapped, the smaller vector the bigger value in map [1,0]
    //  comp            =>    4               ==> this value always is 4
    //  mags            =>    mData           ==> provenance vector norms
    //  a_th            =>    object->agg_th  ==> threshold read from parameters.txt (VectorDiff)
    //  ppCentroid      =>    ppC             ==> it only have "1" as value, possibly to detected multiple affordances in future
    //  startppCentroid =>    startppC        ==> index of position 0-4095 (asociated with ppC), possibly to detected multiple affordances in future
    //  ppCentroidData  =>    ppCData         ==> this ha 4 values: affordance id, orientation, #ofKeypoint, and a value to align
  __global__ void bayesianKernel(float *nn,float *kp, float *pv, float *tp, float *C, int start, int end, int comp, float *weights, float *mags, float a_th, int *ppCentroid, int* startppCentroid, float *ppCentroidData){

    //I think I only need row
    int inner_ele = blockIdx.y*blockDim.y+threadIdx.y;    //This goes 0-2048
    int actual_ele=inner_ele + start; //This goes 0-2048 for now, could get larger
    //Get the actual_ele neighbour and compute vectors and stuff
    float xt= nn[actual_ele*comp+0]-(tp[0]+kp[actual_ele*3+0]);
    float yt= nn[actual_ele*comp+1]-(tp[1]+kp[actual_ele*3+1]);
    float zt= nn[actual_ele*comp+2]-(tp[2]+kp[actual_ele*3+2]);
    for (int i=0;i<ppCentroid[actual_ele];i++)
    {
      int idx=startppCentroid[actual_ele]+i;  //0-2969 for now, could get larger
      int or_id=ppCentroidData[idx*comp+1];
      int pv_id=ppCentroidData[idx*comp+2];


      float angle=or_id*2*PI/8;
      float xpv=pv[idx*3+0]*cos(angle)-pv[idx*3+1]*sin(angle);
      float ypv=sin(angle)*pv[idx*3+0]+cos(angle)*pv[idx*3+1];
      float zpv=pv[idx*3+2];


      float diff=sqrt(((xt-xpv)*(xt-xpv))+((yt-ypv)*(yt-ypv))+((zt-zpv)*(zt-zpv)))/mags[idx];  //This is the difference as proportion of expected magnitude

      //Likelihood is the sample from a normal distribution with mean 0 and std=0.1/weighs;
      float sigma=a_th*(1+weights[idx]);
      
      float likelyhood=expf(-(diff*diff)/(2*sigma*sigma));
      
      C[idx]=likelyhood*weights[idx];


    }
  
  }

  void bayesian_scores(float *nn,float *kp, float *pv, float *tp, float *C, int start, int end, int comp, float *weights, float *mags, float a_th, int *ppCentroid, int* startppCentroid, float *ppCentroidData){
    int maxThreads=128;      //From tables
    int N=end-start;
    dim3 threadsPerBlock(1, maxThreads);  //1x128
    dim3 blocksPerGrid(1, N/maxThreads);  //1x(4096/128) => 1x32
    bayesianKernel<<<blocksPerGrid,threadsPerBlock>>>(nn, kp, pv, tp, C, start, end, comp, weights, mags, a_th, ppCentroid, startppCentroid, ppCentroidData);
    cudaDeviceSynchronize();
  }
