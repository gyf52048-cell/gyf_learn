import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision import datasets, transforms

class Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(1, 32, kernel_size=3, padding=1)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, padding=1)
        self.pool = nn.MaxPool2d(2, 2)
        self.fc1 = nn.Linear(64 * 7 * 7, 128)
        self.fc2 = nn.Linear(128, 10)

    def forward(self, x):
        x = self.pool(F.relu(self.conv1(x)))
        x = self.pool(F.relu(self.conv2(x)))
        x = x.view(x.size(0), -1)
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x

model = Net()
model.load_state_dict(torch.load("mnist_cnn.pth"))
model.eval()

transform = transforms.ToTensor()
test_dataset = datasets.MNIST(root="./data", train=False, download=True, transform=transform)
image, true_label = test_dataset[0]

image_input = image.unsqueeze(0)
with torch.no_grad():
    output = model(image_input)
    predicted = torch.argmax(output, 1).item()

print(f"真实标签: {true_label}，预测标签: {predicted}")

img = transforms.ToPILImage()(image).resize((280, 280))
img.save(f"predict_true{true_label}_pred{predicted}.png")
print("预测图已保存")
