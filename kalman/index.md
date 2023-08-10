# Kalman Filters: High Level Concepts

Fundamentally, the Kalman Filter is an algorithm that solves the _filtering problem_. The filtering problem involves finding the best estimate $\hat{x}(t)$ of some true process $\vec{x}(t)$ given noisy measurements $\vec{z}(t)$. There are 2 sources of stochastic noise: process noise (noise within $\vec{x}(t)$ itself) and measurement noise. 

To simplify the discussion, we will makes some assumptions: 
- Both forms of noise are gaussian and time-invariant. Both are white noise: the process noise covariance matrix $Q$ and measurement noise covariance matrix $R$ are diagonal matrices.
- The process is linear

Under such assumptions we can represent our process and measurements as follows:

$
\vec{x}(k) = F \vec{x}(k-1) + \vec{\eta}(k)
\\
\vec{z}(k) = H \vec{x}(k) + \vec{\xi}(k)
$

Where $\vec{\eta}(k) \sim \mathcal{N}(0, Q)$ and $\vec{\xi}(k) \sim \mathcal{N}(0, R)$ 

{cite}`young2011recursive` (4.44).

The Kalman Filter algorithm can be summarized as follows. At the $k$-th iteration: 

1. Prediction Step
    - Make a prediction $\hat{x}(k | k-1)$ of the value of state $\vec{x}(k)$ based on the process model and previous state estimate $\vec{x}(k-1)$
    - Update covariance $P$, which quantifies the uncertainty of our estimate, based on the previous value of $P$ and the process noise covariance $Q$.
2. Update Step: Incorporating the measurement $\vec{z}(k)$
    - Update the prediction $\hat{x}(k | k-1)$ with the information $\vec{z}(k)$. The estimate $\hat{x}(k)$ will fall along the residual between the prediction and measurement. The uncertainties of the measurement and prediction are used to calculate the Kalman Gain, which scales how far long that residual vector the estimate $\hat{x}(k)$ will fall. For example, if the measurement's uncertainty (quantified by $R$) is much less than the prediction's uncertainty (quantified by $P$), $\hat{x}(k)$ will fall much closer to $\vec{z}(k)$.

The exact formulas for the algorithm outlined above can be found in {cite}`young2011recursive` Chapter 4.4.