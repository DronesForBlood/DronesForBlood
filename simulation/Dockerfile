FROM px4io/px4-dev-ros:2018-09-06

RUN mkdir -p /usr/src/
WORKDIR /usr/src/app/

RUN git clone https://github.com/PX4/Firmware.git


ADD run.sh /usr/src/run.sh
RUN chmod +x /usr/src/run.sh

EXPOSE 14557
EXPOSE 14556
EXPOSE 14560
EXPOSE 11345

ENTRYPOINT ["/usr/src/run.sh"]
