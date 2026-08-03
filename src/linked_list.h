/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file linked_list.h
 * @brief Minimal, intrusive, generic doubly-linked list for plain-C data structures: embed a
 *        linked_list_node_t as the first member of any struct to make it insertable into a
 *        linked_list_t, without any per-node heap allocation of its own.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef LINKED_LIST_H
#define LINKED_LIST_H

#ifdef __cplusplus
extern "C" {
#endif


//_____ D E F I N I T I O N S ______________________________________________

/** Intrusive linked-list link. Place this structure at the start of any data structure to reference properly, so a pointer to that structure can be treated as a linked_list_node_t*. */
typedef struct
{
    void                *prev;                  //!< previous object in the linked list; 0 indicates this is the head
    void                *nextCt;                 //!< next object in the linked list; 0 indicates this is the tail
}linked_list_node_t;

/** Head/tail handle for a doubly-linked list of linked_list_node_t-derived nodes. */
typedef struct
{
    linked_list_node_t  *head;                  //!< head of the linked list, or 0 if the list is empty
    linked_list_node_t  *tail;                  //!< tail of the linked list, or 0 if the list is empty
}linked_list_t;

//_____ P R O T O T Y P E S ________________________________________________

/**
 * Empties a linked list. Does not free/modify the nodes themselves.
 * @param ll the linked list to clear
 */
void linkedListClear(linked_list_t *ll);

/**
 * Inserts a node at the head of a linked list.
 * @param ll the linked list to insert into
 * @param newNode the node to insert; its prev/nextCt fields are overwritten
 */
void linkedListInsertAtHead(linked_list_t *ll, linked_list_node_t *newNode);

/**
 * Inserts a node immediately before an existing node in a linked list.
 * @param ll the linked list containing node
 * @param node the existing node to insert before
 * @param newNode the node to insert; its prev/nextCt fields are overwritten
 */
void linkedListInsertBefore(linked_list_t *ll, linked_list_node_t *node, linked_list_node_t *newNode);

/**
 * Removes a node from a linked list, relinking its neighbors and updating head/tail as needed.
 * @param ll the linked list containing node
 * @param node the node to remove
 */
void linkedListRemove(linked_list_t *ll, linked_list_node_t *node);


#ifdef __cplusplus
}
#endif

#endif // LINKED_LIST_H
