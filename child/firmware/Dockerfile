FROM espressif/idf:release-v6.0

ENV PATH="/opt/esp/python_env/idf6.0_py3.12_env/bin:${PATH}"

RUN python -m pip install --no-cache-dir \
    catkin_pkg \
    colcon-common-extensions \
    lark \
    empy==3.3.4

WORKDIR /workspace/child/firmware

CMD ["bash"]