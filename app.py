import streamlit as st
import bi_rrt
import pf


st.title("Path Planning algorithms")
st.header("Bi-RRT")

max_iter = st.slider('Iterations', 0, 1000, 100)  # min: 0h, max: 23h, default: 17h
fig_birrt = bi_rrt.main(max_iter)
st.pyplot(fig_birrt)

st.header("APF")
grid_size = st.slider("Grid size", 0.0, 5.0, 0.5)
func = st.radio("Attractive Potential Function", ['Paraboloid', 'Conical'])
K = st.slider("Attractive Potential Gain", 0.1, 10.0, 1.0)
ETA = st.slider("Repulsive Potential Gain", 500, 10000, 5000)
fig_apf = pf.main(grid_size, func, ETA, K)
st.pyplot(fig_apf)
