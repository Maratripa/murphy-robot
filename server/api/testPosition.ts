export default defineEventHandler(async (event) => {
    const body = await readBody(event)
    console.log("Llamaste a testPosition")
    console.log(body)
    await $fetch('http://localhost:6969', {
        method: 'POST',
        body: {
            command: 'testPosition',
            q1: body.q1,
            q2: body.q2,
            z: body.z
        }
    })
    return "Llamaste a testPosition"	
})
