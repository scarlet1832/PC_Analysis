import xlsxwriter as xw

def WritetToExcel(data, fileName):
    workbook = xw.Workbook(fileName + '.xlsx')
    worksheet1 = workbook.add_worksheet('sheet1')
    worksheet1.activate()
    i = 1
    for j in range(len(data)):
        if len(data[j]) != 1:
            # print(len(data[j]))
            for k in range(0, len(data[j]) - 1, 2):
                row = 'A' + str(i)
                worksheet1.write_row(row, [str(data[j][k]), str(data[j][k + 1])])
                i += 1

    workbook.close()


if __name__ == '__main__':
    testData = [
     [
      'NoRsult'],
     [
      'intensity', [5, 3, 4, 6, 87, 2342, 345, 56543]],
     [
      'asd', 123, 'asd', 123]]
    fileName = '测试result.xlsx'
    WritetToExcel(testData, fileName)
    print('end')