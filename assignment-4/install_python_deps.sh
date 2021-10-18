# update pip if need be
pip3 install --user -U pip

# install tensorflow
pip3 install -U --user numpy tensorflow==2.6.0 tensorboard==2.7.0 scipy matplotlib scikit-learn gdown

# install additional deps
sudo apt-get install python-tk  # required by tensorflow
