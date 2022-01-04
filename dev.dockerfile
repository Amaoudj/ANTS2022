FROM ubuntu:20.04

ARG PYCHARM_VERSION
ARG PYCHARM_BUILD

LABEL maintainer ""

ENV TZ=Europe/Copenhagen
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install --no-install-recommends -y \
  python3 python3-dev python3-setuptools python3-pip \
  git openssh-client less curl \
  universal-ctags  \
  python3-venv \
  python3-matplotlib \
  python3-pygraphviz \
  python3-tk \
  ffmpeg \
  gcc \
  mc \
  libxtst-dev libxext-dev libxrender-dev libfreetype6-dev \
  libfontconfig1 libgtk2.0-0 libxslt1.1 libxxf86vm1 

RUN ln -s /usr/bin/python3 /usr/bin/python

RUN useradd -ms /bin/bash developer

# PyCharm:

ARG pycharm_source=https://download.jetbrains.com/python/pycharm-community-${PYCHARM_BUILD}.tar.gz
ARG pycharm_local_dir=.PyCharmCE${PYCHARM_VERSION}

RUN apt-get install -y --no-install-recommends libxss1 libnss3 libatk-bridge2.0-dev libgbm-dev

RUN rm -rf /var/lib/apt/lists/* 

WORKDIR /opt/pycharm

RUN curl -fsSL ${pycharm_source} -o /opt/pycharm/installer.tgz && \
   tar --strip-components=1 -xzf installer.tgz && \
   rm installer.tgz

# Development installment
RUN python -m pip install flake8
RUN python -m pip install PyQt5

USER developer
ENV HOME /home/developer
ENV PATH="/home/developer/.local/bin:/opt/pycharm/bin:$PATH"
WORKDIR /home/developer

#RUN mkdir /home/developer/.PyCharm \
#    && ln -sf /home/developer/.PyCharm /home/developer/$pycharm_local_dir

# Installing dependices
COPY requirement.txt /home/developer
RUN python -m pip install -r requirement.txt


# CMD [ "/opt/pycharm/bin/pycharm.sh" ]
CMD bash