FROM python:3.10

# make sure output is always on screen
ENV PYTHONUNBUFFERED 1

# create our working directory
RUN mkdir -p /opt/

# copy files to target, we copy everything, since if we filter
# we will stuck with docker problem of parent directory ignored
COPY rotem_sela_backend  /opt/rotem_sela

# lets install required libraries
WORKDIR /opt/rotem_sela
RUN apt update
RUN apt install -y libgl1 libusb-1.0-0
RUN pip install --upgrade pip
RUN pip install -r requirements.txt

# entry point
CMD ["python","-m","main"]
