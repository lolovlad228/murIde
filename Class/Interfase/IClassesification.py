from abc import ABC, abstractmethod


class IClassesificationType(ABC):

    @abstractmethod
    def cycle(self, img):
        pass

    @abstractmethod
    def square(self, img):
        pass

    @abstractmethod
    def square_with_a_hole(self, img):
        pass