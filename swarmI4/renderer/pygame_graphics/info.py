import csv
import pandas
class Info:

    def __init__(self,text, is_title:bool=False):
        """
        class for storing info
        """
        self.is_title = is_title # True for title , False for normal text
        self.text =text

    @staticmethod
    def store(data,file_path):
        """
        store the info in a csv file
        """
        dataframe = pandas.DataFrame(data)
        print(dataframe)
        with open(file_path, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(data)


