import torch
import torch.nn as nn
import torch.nn.functional as F
import sys

class Net(nn.Module):
    def __init__(self, input_channels):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(input_channels, 20, 5)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(20, 50, 5)
        self.fc1 = nn.Linear(50 * 12 * 12, 500)
        self.fc2 = nn.Linear(500, 2)

    def forward(self, x):
        x = self.pool(F.relu(self.conv1(x)))
        x = self.pool(F.relu(self.conv2(x)))
        x = x.view(-1, 50*12*12)
        #x = x.view(-1, x.shape[1] * x.shape[2] * x.shape[3])
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x

class lucaNet(nn.Module):
    def __init__(self, input_channels):
        super(lucaNet, self).__init__()
        self.conv1 = nn.Conv2d(input_channels, 20, 5)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(20, 50, 5)
        self.conv3 = nn.Conv2d(50, 300, 3)
#        self.fc1 = nn.Linear(50 * 12 * 12, 500)
        self.fc1 = nn.Linear(300 * 5 * 5, 1000)
#        self.fc2 = nn.Linear(500, 2)
        self.fc2 = nn.Linear(1000, 2)

    def forward(self, x):
        x = self.pool(F.relu(self.conv1(x)))
        x = self.pool(F.relu(self.conv2(x)))
        x = self.pool(F.relu(self.conv3(x)))
        x = x.view(-1, x.shape[1] * x.shape[2] * x.shape[3])
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x

class lucaNetRedu(nn.Module):
    def __init__(self, input_channels):
        super(lucaNetRedu, self).__init__()
        self.conv1 = nn.Conv2d(input_channels, 20, 5)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(20, 50, 5)
        self.conv3 = nn.Conv2d(50, 100, 3)
#        self.fc1 = nn.Linear(50 * 12 * 12, 500)
        self.fc1 = nn.Linear(100 * 5 * 5, 500)
#        self.fc2 = nn.Linear(500, 2)
        self.fc2 = nn.Linear(500, 2)

    def forward(self, x):
        x = self.pool(F.relu(self.conv1(x)))
        x = self.pool(F.relu(self.conv2(x)))
        x = self.pool(F.relu(self.conv3(x)))
        x = x.view(-1, x.shape[1] * x.shape[2] * x.shape[3])
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x

class lucaNetFC(nn.Module):
    def __init__(self, input_channels):
        super(lucaNetFC, self).__init__()
        self.conv1 = nn.Conv2d(input_channels, 20, 5)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(20, 50, 5)
        self.conv3 = nn.Conv2d(50, 100, 3)
        self.fc1 = nn.Linear((100 * 5 * 5) + (50 * 12 * 12) + (20 * 28 * 28), 500)
        self.fc2 = nn.Linear(500, 2)

    def forward(self, x):
        x_c1 = self.pool(F.relu(self.conv1(x)))
        x_c2 = self.pool(F.relu(self.conv2(x_c1)))
        x_c3 = self.pool(F.relu(self.conv3(x_c2)))
	x_c1 = x_c1.view(-1, x_c1.shape[1] * x_c1.shape[2] * x_c1.shape[3])
	x_c2 = x_c2.view(-1, x_c2.shape[1] * x_c2.shape[2] * x_c2.shape[3])
	x_c3 = x_c3.view(-1, x_c3.shape[1] * x_c3.shape[2] * x_c3.shape[3])
	x = torch.cat((x_c1,x_c2,x_c3),1)
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x

class lucaNet2(nn.Module):
    def __init__(self, input_channels):
        super(lucaNet2, self).__init__()
        self.conv1 = nn.Conv2d(input_channels, 20, 5)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(20, 50, 5)
        self.conv3 = nn.Conv2d(50, 300, 3)
#        self.fc1 = nn.Linear(50 * 12 * 12, 500)
        self.fc1 = nn.Linear(300 * 12 * 12, 1000)
#        self.fc2 = nn.Linear(500, 2)
        self.fc2 = nn.Linear(1000, 2)

    def forward(self, x):
        x = F.relu(self.conv1(x))
	#print(x.size())
        x = self.pool(F.relu(self.conv2(x)))
	#print(x.size())
        x = self.pool(F.relu(self.conv3(x)))
	#print(x.size())
        x = x.view(-1, x.shape[1] * x.shape[2] * x.shape[3])
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x

class lucaNet2Redu(nn.Module):
    def __init__(self, input_channels):
        super(lucaNet2Redu, self).__init__()
        self.conv1 = nn.Conv2d(input_channels, 20, 5)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(20, 50, 5)
        self.conv3 = nn.Conv2d(50, 100, 3)
#        self.fc1 = nn.Linear(50 * 12 * 12, 500)
        self.fc1 = nn.Linear(100 * 12 * 12, 500)
#        self.fc2 = nn.Linear(500, 2)
        self.fc2 = nn.Linear(500, 2)

    def forward(self, x):
        x = F.relu(self.conv1(x))
	#print(x.size())
        x = self.pool(F.relu(self.conv2(x)))
	#print(x.size())
        x = self.pool(F.relu(self.conv3(x)))
	#print(x.size())
        x = x.view(-1, x.shape[1] * x.shape[2] * x.shape[3])
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x

class lucaNet2FC(nn.Module):
    def __init__(self, input_channels):
        super(lucaNet2FC, self).__init__()
        self.conv1 = nn.Conv2d(input_channels, 20, 5)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(20, 50, 5)
        self.conv3 = nn.Conv2d(50, 100, 3)
#        self.fc1 = nn.Linear(50 * 12 * 12, 500)
        self.fc1 = nn.Linear((100 * 12 * 12) + (50 * 26 * 26), 500)
#        self.fc2 = nn.Linear(500, 2)
        self.fc2 = nn.Linear(500, 2)

    def forward(self, x):
        x_c1 = F.relu(self.conv1(x))
        x_c2 = self.pool(F.relu(self.conv2(x_c1)))
        x_c3 = self.pool(F.relu(self.conv3(x_c2)))
	x_c1 = x_c1.view(-1, x_c1.shape[1] * x_c1.shape[2] * x_c1.shape[3])
	x_c2 = x_c2.view(-1, x_c2.shape[1] * x_c2.shape[2] * x_c2.shape[3])
	x_c3 = x_c3.view(-1, x_c3.shape[1] * x_c3.shape[2] * x_c3.shape[3])
	x = torch.cat((x_c2,x_c3),1)
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x


class lucaNet3(nn.Module):
    def __init__(self, input_channels):
        super(lucaNet3, self).__init__()
        self.conv1 = nn.Conv2d(input_channels, 20, 5)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(20, 50, 5)
        self.conv3 = nn.Conv2d(50, 300, 5)
#        self.fc1 = nn.Linear(50 * 12 * 12, 500)
        self.fc1 = nn.Linear(300 * 11 * 11, 1000)
#        self.fc2 = nn.Linear(500, 2)
        self.fc2 = nn.Linear(1000, 2)

    def forward(self, x):
        x = F.relu(self.conv1(x))
	#print(x.size())
        x = self.pool(F.relu(self.conv2(x)))
	#print(x.size())
        x = self.pool(F.relu(self.conv3(x)))
	#print(x.size())
        x = x.view(-1, x.shape[1] * x.shape[2] * x.shape[3])
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x

class lucaNet3Redu(nn.Module):
    def __init__(self, input_channels):
        super(lucaNet3Redu, self).__init__()
        self.conv1 = nn.Conv2d(input_channels, 20, 5)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(20, 50, 5)
        self.conv3 = nn.Conv2d(50, 100, 5)
#        self.fc1 = nn.Linear(50 * 12 * 12, 500)
        self.fc1 = nn.Linear(100 * 11 * 11, 500)
#        self.fc2 = nn.Linear(500, 2)
        self.fc2 = nn.Linear(500, 2)

    def forward(self, x):
        x = F.relu(self.conv1(x))
	#print(x.size())
        x = self.pool(F.relu(self.conv2(x)))
	#print(x.size())
        x = self.pool(F.relu(self.conv3(x)))
	#print(x.size())
        x = x.view(-1, x.shape[1] * x.shape[2] * x.shape[3])
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x

input_channels = int(sys.argv[3])
net = Net(input_channels)
#net = lucaNetRedu(input_channels)
#net = lucaNetFC(input_channels)
#net = lucaNet2FC(input_channels)
net.load_state_dict(torch.load(sys.argv[1]))
print(net)

dummy_input = torch.randn(1, input_channels, 60, 60)
torch.onnx.export(net, dummy_input, sys.argv[2], verbose=True)
