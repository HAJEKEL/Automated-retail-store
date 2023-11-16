product_list = []

product1 = ["april_tag_cube_8",  "hagelslag", 0.5]
product2 = ["april_tag_cube_23", "hagelslag", 0.1]
product3 = ["april_tag_cube_15", "yoghurt", 1.0]
product4 = ["april_tag_cube_24", "yoghurt", 0.5]

product_list.append(product1)
product_list.append(product2)
product_list.append(product3)
product_list.append(product4)

weight_full_hag = 0.5
weight_full_yog = 1.0

for product in product_list:
    #INSERT CODE TO CREATE INSTANCE
    print(product[0])
    if product[1] == "hagelslag":
        print("Is hagelslag)")
        print("not refrigirated")
        if product[2] >= weight_full_hag:
            print("full")
        elif product[2] < weight_full_hag:
            print("empty")

    elif product[1] == "yoghurt":
        print("Is yoghurt)")
        print("refrigirated")
        if product[2] >= weight_full_yog:
            print("full")
        elif product[2] < weight_full_yog:
            print("empty")