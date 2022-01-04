image             ?= pycharm:dev
container_id_file ?= ./container_id
name              ?= pycharm_development
pycharm_version   ?= 2021.2
pycharm_build     ?= 2021.2.1


build:
	docker build -t $(image) --build-arg PYCHARM_VERSION=$(pycharm_version) --build-arg=PYCHARM_BUILD=$(pycharm_build) -f dev.dockerfile .

buildrm:
	docker rmi $(image)

$(container_id_file):
	xhost local:docker
	mkdir -p $(HOME)/.cache/JetBrains/PyCharmCE$(pycharm_version)
	mkdir -p $(HOME)/.config/JetBrains/PyCharmCE$(pycharm_version)
	mkdir -p $(HOME)/.local/share/JetBrains
	mkdir -p $(HOME)/.java
	docker create -it \
		--cidfile $(container_id_file) \
		-e DISPLAY \
		--workdir=/home/developer/package \
		-v $(shell pwd):/home/developer/package \
		-v $(HOME)/.emacs.d:/home/developer/.emacs.d \
		-v $(HOME)/.java:/home/developer/.java \
		-v $(HOME)/.config/JetBrains:/home/developer/.config/JetBrains \
		-v $(HOME)/.cache/JetBrains/PyCharmCE$(pycharm_version):/home/developer/.cache/JetBrains/PyCharmCE$(pycharm_version) \
		-v $(HOME)/.local/share/JetBrains:/home/developer/.local/share/JetBrains \
		-e QT_X11_NO_MITSHM=1 \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v ~/.Xauthority:/root/.Xauthority \
		-v /run/user/1000:/run/user/1000 \
		-e XDG_RUNTIME_DIR \
		-t \
		--name $(name) \
		$(image) 

start: $(container_id_file)
	xhost local:docker
	docker container start -ia $(shell cat $(container_id_file) )

stop:
	docker container stop $(shell cat $(container_id_file) )

rm:
	docker container stop $(shell cat $(container_id_file) )
	docker container rm $(shell cat $(container_id_file) )
	rm $(container_id_file)

enter: $(container_id_file)
	docker exec -it $(shell cat $(container_id_file)) bash

enter-root: $(container_id_file)
	docker exec -u 0 -it $(shell cat $(container_id_file)) bash

echo:
	echo /.cache/JetBrains/PyCharmCE=$(PYCHARM_VERSION)


.PHONY: build buildrm create start stop rm enter browser
